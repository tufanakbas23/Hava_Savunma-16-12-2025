import os
os.environ["OPENCV_VIDEOIO_PRIORITY_MSMF"] = "0"
os.environ["OPENCV_VIDEOIO_PRIORITY_DSHOW"] = "100"

import sys
import time
import threading
import json
import cv2
import numpy as np
from collections import deque
from concurrent.futures import ThreadPoolExecutor

from PySide6.QtCore import Qt, Signal, Slot, QTimer
from PySide6.QtGui import QImage, QPixmap, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QComboBox, QHBoxLayout, QVBoxLayout, QGroupBox, QGridLayout,
    QPlainTextEdit, QTabWidget, QDoubleSpinBox, QScrollArea, QFrame,
    QSizePolicy,
)

# ─── Proje yolunu Python path'e ekle ───────────────────────────────
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from camera.zed_camera import ZedCamera
from ultralytics import YOLO

# ==================== SABİTLER ====================

ENGINE_PATH   = os.path.join(PROJECT_ROOT, "balon_baska.engine")
LOGITECH_IDX  = 0          # Eğer 0 PC kamerası açarsa, 1 veya 2 deneyebilirsiniz. USB portuna göre değişir.
USE_ZED       = False      # True: ZED'i açmaya çalışır, False: ZED kodda tamamen kapalı kalır
YOLO_CONF     = 0.35
YOLO_IMGSZ    = 480
DISPLAY_W     = 640
DISPLAY_H     = 480

# ==================== KAMERA WORKER ====================

class DualCameraWorker:
    """
    ZED (Kamera 1) + Logitech (Kamera 2) aynı anda yakalayan,
    YOLO inference yapan ve sonuçları UI'a gönderen worker.
    Her iki kamera ThreadPoolExecutor ile paralel çalışır.
    """

    def __init__(self, on_frames, on_log):
        """
        on_frames(zed_vis, logi_vis, fps, depth_str) — UI güncellemesi
        on_log(msg)                                  — log satırı
        """
        self._on_frames = on_frames
        self._on_log    = on_log
        self._stop_evt  = threading.Event()
        self._thread    = None

        self.zed_cam  = None
        self.logi_cap = None
        self.model    = None

        self.fps_times = deque(maxlen=30)

    # ── Başlat ──────────────────────────────────────────────────────
    def start(self):
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ── Durdur ──────────────────────────────────────────────────────
    def stop(self):
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=5)
        self._thread = None

    # ── Ana döngü ───────────────────────────────────────────────────
    def _run(self):
        # — Model yükle —
        try:
            self.model = YOLO(ENGINE_PATH)
            self._on_log("✅ YOLO hazır")
        except Exception as e:
            self._on_log(f"❌ YOLO yüklenemedi: {e}")
            return

        # — ZED kamerayı aç (USE_ZED açıksa) —
        if USE_ZED:
            try:
                self.zed_cam = ZedCamera(resolution="HD720", fps=30,
                                         depth_mode="PERFORMANCE", max_depth_m=20.0)
                self.zed_cam.open()
            except Exception as e:
                self._on_log(f"❌ ZED açılamadı: {e}")
                self.zed_cam = None
        else:
            self.zed_cam = None

        # — Logitech kamerayı aç —
        try:
            self.logi_cap = cv2.VideoCapture(LOGITECH_IDX, cv2.CAP_DSHOW)
            self.logi_cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
            self.logi_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.logi_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not self.logi_cap.isOpened():
                self.logi_cap = None
            else:
                self._on_log("✅ Kamera açıldı")
        except Exception as e:
            self._on_log(f"❌ Logitech hatası: {e}")
            self.logi_cap = None

        if self.zed_cam is None and self.logi_cap is None:
            self._on_log("❌ Hiç kamera açılamadı, durduruluyor")
            return

        # — Paralel kamera çekimi için executor —
        executor = ThreadPoolExecutor(max_workers=2)

        def grab_zed():
            if self.zed_cam:
                return self.zed_cam.grab_frame()   # (bgr, depth_mat)
            return None, None

        def grab_logi():
            if self.logi_cap:
                ret, frame = self.logi_cap.read()
                return frame if ret else None
            return None

        # Pipeline prime
        fut_zed  = executor.submit(grab_zed)
        fut_logi = executor.submit(grab_logi)

        while not self._stop_evt.is_set():
            # Sonuçları al
            zed_result = fut_zed.result()
            logi_frame = fut_logi.result()

            # Paralel olarak sonraki frame'leri hemen başlat
            fut_zed  = executor.submit(grab_zed)
            fut_logi = executor.submit(grab_logi)

            rgb_zed, depth_mat = zed_result

            # Görüntüleri boyutlandır
            if rgb_zed is None and logi_frame is None:
                continue

            frames_for_yolo = []
            has_zed  = rgb_zed   is not None
            has_logi = logi_frame is not None

            # TRT modelimiz sabit batch=2 bekliyor (Expected [2,3,480,480]).
            # Bu yüzden eğer tek kamera çalışıyorsa, listeyi 2 elemana tamamlamamız şart!
            if has_zed:
                frames_for_yolo.append(rgb_zed)
            if has_logi:
                frames_for_yolo.append(logi_frame)

            # Liste eksikse siyah frame (padding) ekleyelim ki TRT çökmesin
            while len(frames_for_yolo) > 0 and len(frames_for_yolo) < 2:
                # Elimizdeki ilk frame'in boyutlarında siyah bir frame ekle
                blank = np.zeros_like(frames_for_yolo[0])
                frames_for_yolo.append(blank)

            res_zed = None
            res_logi = None

            if len(frames_for_yolo) == 2:
                try:
                    # Batch (2 frame) olarak gönder
                    results = self.model.predict(
                        source=frames_for_yolo,
                        imgsz=YOLO_IMGSZ,
                        conf=YOLO_CONF,
                        device=0,
                        half=True,
                        verbose=False,
                    )
                    # Eğer ZED açıksa 0. index ZED'dir. Değilse Logitech 0. indextedir.
                    if has_zed:
                        res_zed = results[0]
                        if has_logi:
                            res_logi = results[1]
                    else:
                        if has_logi:
                            res_logi = results[0]
                except Exception as e:
                    self._on_log(f"YOLO Inference Hatası: {e}")

            # — ZED görselleştirme —
            zed_vis = None
            depth_str = "—"
            if has_zed and res_zed is not None:
                zed_vis = rgb_zed.copy()
                for box in res_zed.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cls_id = int(box.cls[0].cpu().numpy())
                    conf   = float(box.conf[0].cpu().numpy())
                    label  = f"{res_zed.names[cls_id]} {conf:.2f}"
                    cv2.rectangle(zed_vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(zed_vis, label, (x1, max(y1 - 5, 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
                    # Derinlik
                    if depth_mat is not None and self.zed_cam:
                        d = self.zed_cam.get_depth_at_bbox(depth_mat, x1, y1, x2, y2)
                        if d:
                            depth_str = f"{d:.1f} m"
                            cv2.putText(zed_vis, f"{d:.1f}m",
                                        (x1, y2 + 16),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

            # — Logitech görselleştirme —
            logi_vis = None
            if has_logi and res_logi is not None:
                logi_vis = logi_frame.copy()
                for box in res_logi.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cls_id = int(box.cls[0].cpu().numpy())
                    conf   = float(box.conf[0].cpu().numpy())
                    label  = f"{res_logi.names[cls_id]} {conf:.2f}"
                    cv2.rectangle(logi_vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(logi_vis, label, (x1, max(y1 - 5, 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)

            # — FPS hesabı —
            self.fps_times.append(time.time())
            avg_fps = 0.0
            if len(self.fps_times) >= 2:
                avg_fps = (len(self.fps_times) - 1) / (
                    self.fps_times[-1] - self.fps_times[0]
                )

            # — UI callback —
            self._on_frames(zed_vis, logi_vis, avg_fps, depth_str)

        # — Temizlik —
        executor.shutdown(wait=False)
        if self.zed_cam:
            self.zed_cam.close()
        if self.logi_cap:
            self.logi_cap.release()


# ==================== RENK KALİBRASYON WİDGET ====================

class ColorCalibrationWidget(QWidget):
    log_signal = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.clicked_values = []
        self.current_color  = "red"
        self.calibrations   = {
            "red":   {"L_center": 60, "A_center": 30,  "B_center": 20,  "radius_ab": 15},
            "blue":  {"L_center": 50, "A_center": -20, "B_center": -25, "radius_ab": 15},
            "green": {"L_center": 60, "A_center": -30, "B_center": 30,  "radius_ab": 15},
        }
        self.init_ui()
        self.load_calibration()

    def init_ui(self):
        layout = QVBoxLayout(self)
        info = QLabel("Stage durdukken video uzerine 10 kez tiklayin")
        info.setStyleSheet("color: #888; font-weight: bold; font-size: 11pt;")
        layout.addWidget(info)

        ctrl_layout = QHBoxLayout()
        ctrl_layout.addWidget(QLabel("Renk:"))
        self.color_combo = QComboBox()
        self.color_combo.addItems(["red", "blue", "green"])
        self.color_combo.currentTextChanged.connect(self.on_color_changed)
        ctrl_layout.addWidget(self.color_combo)
        self.lbl_samples = QLabel("Ornek: 0/10")
        ctrl_layout.addWidget(self.lbl_samples)
        btn_clear = QPushButton("Temizle")
        btn_clear.clicked.connect(self.clear_samples)
        ctrl_layout.addWidget(btn_clear)
        ctrl_layout.addStretch()
        layout.addLayout(ctrl_layout)

        calc_box    = QGroupBox("Hesaplanan Degerler")
        calc_layout = QGridLayout(calc_box)
        self.lbl_l  = QLabel("L: -")
        self.lbl_a  = QLabel("A: -")
        self.lbl_b  = QLabel("B: -")
        self.lbl_r  = QLabel("Radius: -")
        calc_layout.addWidget(self.lbl_l, 0, 0)
        calc_layout.addWidget(self.lbl_a, 1, 0)
        calc_layout.addWidget(self.lbl_b, 2, 0)
        calc_layout.addWidget(self.lbl_r, 3, 0)
        layout.addWidget(calc_box)

        btn_layout = QHBoxLayout()
        btn_save   = QPushButton("Kaydet")
        btn_load   = QPushButton("Yukle")
        btn_export = QPushButton("Kod Cikar")
        btn_save.clicked.connect(self.save_calibration)
        btn_load.clicked.connect(self.load_calibration)
        btn_export.clicked.connect(self.export_code)
        btn_layout.addWidget(btn_save)
        btn_layout.addWidget(btn_load)
        btn_layout.addWidget(btn_export)
        layout.addLayout(btn_layout)
        layout.addStretch()
        self.update_display()

    def handle_click(self, x, y, frame):
        if frame is None or frame.size == 0:
            return
        h, w = frame.shape[:2]
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        lab  = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        L, A, B = lab[y, x]
        self.clicked_values.append([float(L), float(A), float(B)])
        self.lbl_samples.setText(f"Ornek: {len(self.clicked_values)}/10")
        if len(self.clicked_values) == 10:
            self.auto_calculate()

    def auto_calculate(self):
        if len(self.clicked_values) < 3:
            return
        vals       = np.array(self.clicked_values)
        L_center   = float(np.mean(vals[:, 0]))
        A_center   = float(np.mean(vals[:, 1]))
        B_center   = float(np.mean(vals[:, 2]))
        dists_ab   = np.sqrt((vals[:, 1] - A_center)**2 + (vals[:, 2] - B_center)**2)
        radius_ab  = float(np.max(dists_ab) + 5)
        self.calibrations[self.current_color] = {
            "L_center": L_center, "A_center": A_center,
            "B_center": B_center, "radius_ab": radius_ab,
        }
        self.update_display()
        self.log_signal.emit(f"{self.current_color.upper()} kalibrasyon tamamlandi")

    def update_display(self):
        cal = self.calibrations[self.current_color]
        self.lbl_l.setText(f"L: {cal['L_center']:.1f}")
        self.lbl_a.setText(f"A: {cal['A_center']:.1f}")
        self.lbl_b.setText(f"B: {cal['B_center']:.1f}")
        self.lbl_r.setText(f"Radius: {cal['radius_ab']:.1f}")

    def on_color_changed(self, color):
        self.current_color  = color
        self.clicked_values = []
        self.lbl_samples.setText("Ornek: 0/10")
        self.update_display()

    def clear_samples(self):
        self.clicked_values = []
        self.lbl_samples.setText("Ornek: 0/10")

    def save_calibration(self):
        with open("color_calibration.json", "w") as f:
            json.dump(self.calibrations, f, indent=2)
        self.log_signal.emit("color_calibration.json kaydedildi")

    def load_calibration(self):
        try:
            with open("color_calibration.json", "r") as f:
                self.calibrations = json.load(f)
            self.update_display()
            self.log_signal.emit("color_calibration.json yuklendi")
        except FileNotFoundError:
            pass

    def export_code(self):
        cal = self.calibrations[self.current_color]
        print("\n" + "=" * 60)
        print(f"KALIBRASYON KODU - {self.current_color.upper()}")
        print("=" * 60)
        print(f"self.{self.current_color}_center = np.array([{cal['A_center']:.1f}, {cal['B_center']:.1f}])")
        print(f"self.max_dist_{self.current_color} = {cal['radius_ab']:.1f}")
        print("=" * 60 + "\n")
        self.log_signal.emit(f"{self.current_color.upper()} kodu konsola yazdirildi")


# ==================== PD AYARLARI WİDGET ====================

class PDTuningWidget(QWidget):
    params_changed = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        info   = QLabel("PD Kontrolor Parametreleri")
        info.setStyleSheet("font-weight: bold; font-size: 11pt;")
        layout.addWidget(info)

        kp_layout = QHBoxLayout()
        kp_layout.addWidget(QLabel("Kp (Oransal):"))
        self.spin_kp = QDoubleSpinBox()
        self.spin_kp.setRange(0.001, 1.0)
        self.spin_kp.setSingleStep(0.001)
        self.spin_kp.setValue(0.02)
        self.spin_kp.setDecimals(3)
        kp_layout.addWidget(self.spin_kp)
        layout.addLayout(kp_layout)

        kd_layout = QHBoxLayout()
        kd_layout.addWidget(QLabel("Kd (Turev):"))
        self.spin_kd = QDoubleSpinBox()
        self.spin_kd.setRange(0.0, 0.1)
        self.spin_kd.setSingleStep(0.001)
        self.spin_kd.setValue(0.005)
        self.spin_kd.setDecimals(3)
        kd_layout.addWidget(self.spin_kd)
        layout.addLayout(kd_layout)

        btn_apply = QPushButton("Uygula")
        btn_apply.clicked.connect(self.apply_params)
        layout.addWidget(btn_apply)
        layout.addStretch()

    def apply_params(self):
        self.params_changed.emit({"kp": self.spin_kp.value(), "kd": self.spin_kd.value()})


# ==================== YARDIMCI ====================

def _telem_card(title: str) -> tuple:
    lbl_title = QLabel(title)
    lbl_title.setStyleSheet(
        "color: #7a7a8a; font-size: 8pt; font-weight: normal; letter-spacing: 1px;"
    )
    lbl_title.setAlignment(Qt.AlignLeft)
    lbl_value = QLabel("—")
    lbl_value.setStyleSheet("color: #f0f0f0; font-size: 15pt; font-weight: bold;")
    lbl_value.setAlignment(Qt.AlignLeft)
    return lbl_title, lbl_value


def _bgr_to_qpixmap(bgr_frame, label_size) -> QPixmap:
    """NumPy BGR frame → QPixmap (label boyutuna sığdır)."""
    rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    qimg = QImage(rgb.data.tobytes(), w, h, ch * w, QImage.Format_RGB888)
    pix  = QPixmap.fromImage(qimg)
    return pix.scaled(label_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)


# ==================== ANA PENCERE ====================

class MainWindow(QMainWindow):
    # Qt sinyali — worker thread'den UI thread'e güvenli geçiş
    _frame_signal = Signal(object, object, float, str)
    _log_signal   = Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hava Savunma Sistemi — ZED | Logitech")

        self.worker           = None
        self._running         = False
        self.active_stage     = 1
        self.calibration_mode = False
        self.current_frame_bgr = None   # ZED kameradan son frame (kalibrasyon için)

        # Sinyal bağlantıları
        self._frame_signal.connect(self._on_frames_slot)
        self._log_signal.connect(self.append_log)

        self.init_ui()
        self.setup_shortcuts()
        self.apply_styles()

    # ── UI kurulum ──────────────────────────────────────────────────
    def init_ui(self):
        central     = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # ── Logo + İki kamera yan yana ──────────────────────────
        cameras_container = QWidget()
        cameras_outer     = QVBoxLayout(cameras_container)
        cameras_outer.setContentsMargins(0, 0, 0, 0)
        cameras_outer.setSpacing(4)

        logo_row = QHBoxLayout()
        self.logo_label = QLabel()
        self.logo_label.setAlignment(Qt.AlignCenter)
        self.logo_label.setFixedHeight(60)
        self.logo_label.setStyleSheet("background: transparent;")
        logo_path = os.path.join(PROJECT_ROOT, "STNM.png")
        logo_pix  = QPixmap(logo_path)
        if not logo_pix.isNull():
            self.logo_label.setPixmap(logo_pix.scaledToHeight(56, Qt.SmoothTransformation))
        logo_row.addStretch()
        logo_row.addWidget(self.logo_label)
        logo_row.addStretch()
        cameras_outer.addLayout(logo_row)

        cameras_row = QHBoxLayout()
        cameras_row.setSpacing(8)

        # Kamera 1 — ZED
        def make_cam_label(placeholder):
            lbl = QLabel(placeholder)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            lbl.setMinimumSize(600, 200)
            return lbl

        self.video_label   = make_cam_label("ZED\nBEKLENİYOR...")
        self.video_label.setStyleSheet(
            "background:#050508; color:#0f0; border:1px solid #1e1e28; "
            "font-size:14pt; font-family:Consolas;"
        )
        self.video_label.mousePressEvent = self.on_video_click

        # Kamera 2 — Logitech
        self.video_label_2 = make_cam_label("LOGİTECH\nBEKLENİYOR...")
        self.video_label_2.setStyleSheet(
            "background:#050508; color:#0a8; border:1px solid #1e1e28; "
            "font-size:14pt; font-family:Consolas;"
        )

        cameras_row.addWidget(self.video_label,   stretch=1)
        cameras_row.addWidget(self.video_label_2, stretch=1)
        cameras_outer.addLayout(cameras_row)
        main_layout.addWidget(cameras_container, stretch=1)

        # ── Alt: Kontroller + Log ────────────────────────────────
        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout(bottom_widget)
        bottom_layout.setSpacing(8)
        bottom_layout.setContentsMargins(0, 0, 0, 0)

        self.tab_widget = QTabWidget()
        self.tab_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.tab_widget.currentChanged.connect(self.on_tab_changed)
        self.tab_widget.addTab(self.create_control_tab(),     "Kontrol")
        self.tab_widget.addTab(self.create_calibration_tab(), "Kalibrasyon")
        self.tab_widget.addTab(self.create_pd_tab(),          "PD Ayarlari")
        bottom_layout.addWidget(self.tab_widget, stretch=1)

        log_box    = QGroupBox("Log")
        log_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        log_layout = QVBoxLayout(log_box)
        log_layout.setContentsMargins(5, 5, 5, 5)
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(1000)
        log_layout.addWidget(self.log_box)
        bottom_layout.addWidget(log_box, stretch=1)

        main_layout.addWidget(bottom_widget, stretch=0)

    # ── Kontrol sekmesi ─────────────────────────────────────────────
    def create_control_tab(self):
        tab    = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(8)
        layout.setContentsMargins(6, 6, 6, 6)

        sys_box    = QGroupBox("Sistem Kontrolu")
        sys_layout = QVBoxLayout(sys_box)
        sys_layout.setSpacing(8)

        # Kamera bilgi etiketi
        cam_info = QLabel("📷 Kamera 1: ZED   |   📷 Kamera 2: Logitech")
        cam_info.setStyleSheet("color: #6080c0; font-size: 9pt; padding: 2px 0;")
        sys_layout.addWidget(cam_info)

        # Mod seçim butonları
        mode_row = QHBoxLayout()
        mode_row.setSpacing(6)
        self.btn_stage_manuel = QPushButton("Manuel")
        self.btn_stage_suru   = QPushButton("Sürü")
        self.btn_stage_otonom = QPushButton("Dost-Düşman")
        for btn in [self.btn_stage_manuel, self.btn_stage_suru, self.btn_stage_otonom]:
            btn.setMinimumHeight(36)
            btn.setCheckable(True)
            btn.setObjectName("btn_stage_mode")
            mode_row.addWidget(btn, stretch=1)
        self.btn_stage_manuel.setChecked(True)
        self.btn_stage_manuel.clicked.connect(lambda: self.on_stage_btn(1))
        self.btn_stage_suru.clicked.connect(lambda: self.on_stage_btn(2))
        self.btn_stage_otonom.clicked.connect(lambda: self.on_stage_btn(3))
        sys_layout.addLayout(mode_row)

        ss_row = QHBoxLayout()
        ss_row.setSpacing(6)
        self.btn_start = QPushButton("START")
        self.btn_stop  = QPushButton("STOP")
        self.btn_start.setObjectName("btn_start")
        self.btn_stop.setObjectName("btn_stop")
        self.btn_start.setMinimumHeight(42)
        self.btn_stop.setMinimumHeight(42)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        ss_row.addWidget(self.btn_start, stretch=1)
        ss_row.addWidget(self.btn_stop,  stretch=1)
        sys_layout.addLayout(ss_row)
        layout.addWidget(sys_box)

        # Telemetri
        telem_box    = QGroupBox("Telemetri")
        telem_layout = QHBoxLayout(telem_box)
        telem_layout.setContentsMargins(12, 14, 12, 12)
        telem_layout.setSpacing(0)

        t2, self.lbl_depth = _telem_card("LİDAR MESAFE")
        t3, self.lbl_lock  = _telem_card("DURUM")

        for i, (title_lbl, val_lbl) in enumerate(
            [(t2, self.lbl_depth), (t3, self.lbl_lock)]
        ):
            card = QVBoxLayout()
            card.setSpacing(2)
            card.addWidget(title_lbl)
            card.addWidget(val_lbl)
            telem_layout.addLayout(card)
            if i < 1:
                sep = QFrame()
                sep.setFrameShape(QFrame.VLine)
                sep.setStyleSheet("color: #2a2a35; margin: 4px 20px;")
                telem_layout.addWidget(sep)

        layout.addWidget(telem_box)
        layout.addStretch()
        return tab

    def create_calibration_tab(self):
        tab    = QWidget()
        layout = QVBoxLayout(tab)
        self.color_calib_widget = ColorCalibrationWidget(parent=self)
        self.color_calib_widget.log_signal.connect(self.append_log)
        layout.addWidget(self.color_calib_widget)
        return tab

    def create_pd_tab(self):
        tab    = QWidget()
        layout = QVBoxLayout(tab)
        self.pd_widget = PDTuningWidget(parent=self)
        self.pd_widget.params_changed.connect(self.on_pd_changed)
        layout.addWidget(self.pd_widget)
        return tab

    # ── Stiller ─────────────────────────────────────────────────────
    def apply_styles(self):
        self.setStyleSheet("""
QMainWindow, QMainWindow > QWidget {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
        stop:0 #080810, stop:1 #14141e);
}
QWidget { background-color: transparent; color: #d0d0d8;
    font-family: 'Segoe UI', Arial; font-size: 10pt; }
QGroupBox {
    background-color: rgba(255,255,255,0.025);
    border: 1px solid #22222e; border-radius: 6px;
    margin-top: 10px; padding-top: 10px;
    font-weight: bold; color: #808090; font-size: 8pt; letter-spacing: 1px;
}
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }
QPushButton {
    background-color: #14141e; border: 1px solid #2c2c3e;
    border-radius: 5px; padding: 7px 14px; color: #c0c0cc; font-weight: bold;
}
QPushButton:hover { background-color: #1e1e2e; border: 1px solid #4444aa; color: #e0e0ff; }
QPushButton:pressed { background-color: #0c0c16; }
QPushButton:disabled { background-color: #0e0e18; color: #404050; }
QPushButton#btn_start {
    background-color: #081808; border: 1px solid #1a5a1a; color: #55cc55;
}
QPushButton#btn_start:hover {
    background-color: #0e260e; border: 1px solid #22aa22; color: #88ff88;
}
QPushButton#btn_stop {
    background-color: #180808; border: 1px solid #5a1a1a; color: #cc5555;
}
QPushButton#btn_stop:hover {
    background-color: #260e0e; border: 1px solid #aa2222; color: #ff8888;
}
QComboBox {
    background-color: #14141e; border: 1px solid #2c2c3e;
    border-radius: 5px; padding: 5px 8px; color: #c0c0cc;
}
QComboBox:hover { border: 1px solid #4444aa; }
QComboBox::drop-down { border: none; width: 20px; }
QComboBox QAbstractItemView {
    background-color: #14141e; selection-background-color: #28284a;
    border: 1px solid #2c2c3e; color: #c0c0cc;
}
QLabel { color: #c0c0cc; background: transparent; }
QTabWidget::pane {
    border: 1px solid #22222e; border-radius: 5px;
    background-color: rgba(255,255,255,0.015);
}
QTabBar::tab {
    background-color: #0e0e18; border: 1px solid #22222e;
    padding: 7px 18px; margin-right: 2px;
    border-radius: 4px 4px 0 0; color: #707080;
}
QTabBar::tab:selected {
    background-color: #14141e; border-bottom: 2px solid #4444cc;
    color: #aaaaee; font-weight: bold;
}
QTabBar::tab:hover { background-color: #18182a; color: #b0b0dd; }
QScrollBar:vertical { background: #0a0a12; width: 6px; border-radius: 3px; }
QScrollBar::handle:vertical { background: #2a2a44; border-radius: 3px; min-height: 20px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QSpinBox, QDoubleSpinBox {
    background-color: #14141e; border: 1px solid #2c2c3e;
    border-radius: 4px; padding: 4px; color: #c0c0cc;
}
QPlainTextEdit {
    background: #050508; color: #00e676;
    font-family: Consolas, monospace; font-size: 9pt;
    border: none; border-radius: 4px;
}
""")

    # ── Kısayollar ──────────────────────────────────────────────────
    def setup_shortcuts(self):
        QShortcut(QKeySequence("Escape"), self, self.on_stop)

    def on_stage_btn(self, idx: int):
        if self._running:
            self.append_log("UYARI: Once STOP yapin, sonra mod degistirin")
            self.btn_stage_manuel.setChecked(self.active_stage == 1)
            self.btn_stage_suru.setChecked(self.active_stage == 2)
            self.btn_stage_otonom.setChecked(self.active_stage == 3)
            return
        self.active_stage = idx
        self.btn_stage_manuel.setChecked(idx == 1)
        self.btn_stage_suru.setChecked(idx == 2)
        self.btn_stage_otonom.setChecked(idx == 3)
        names = {1: "Manuel", 2: "Sürü", 3: "Otonom Mod"}
        self.append_log(f"{names[idx]} modu secildi")

    def on_tab_changed(self, index):
        self.calibration_mode = (index == 1)
        if not hasattr(self, "log_box"):
            return
        msg = ("Kalibrasyon modu ACIK — ZED video uzerine tiklayin"
               if self.calibration_mode else "Kalibrasyon modu KAPALI")
        self.append_log(msg)

    def on_video_click(self, event):
        if not self.calibration_mode:
            return
        if self._running:
            self.append_log("UYARI: Calisirken kalibrasyon yapilamaz")
            return
        if self.current_frame_bgr is None:
            self.append_log("UYARI: Henuz ZED frame yok")
            return
        lw, lh = self.video_label.width(), self.video_label.height()
        fh, fw = self.current_frame_bgr.shape[:2]
        x = int(event.x() * fw / lw)
        y = int(event.y() * fh / lh)
        self.color_calib_widget.handle_click(x, y, self.current_frame_bgr)

    # ── Log ─────────────────────────────────────────────────────────
    @Slot(str)
    def append_log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_box.appendPlainText(f"[{ts}] {msg}")
        self.log_box.verticalScrollBar().setValue(
            self.log_box.verticalScrollBar().maximum()
        )

    # ── Frame güncelleme (sinyal üzerinden — thread-safe) ───────────
    @Slot(object, object, float, str)
    def _on_frames_slot(self, zed_vis, logi_vis, fps, depth_str):
        """worker thread'den gelen frame'leri UI'a yansıtır."""
        try:
            # ZED frame
            if zed_vis is not None:
                self.current_frame_bgr = zed_vis  # kalibrasyon için tut
                pix = _bgr_to_qpixmap(zed_vis, self.video_label.size())
                self.video_label.setPixmap(pix)

            # Logitech frame
            if logi_vis is not None:
                pix2 = _bgr_to_qpixmap(logi_vis, self.video_label_2.size())
                self.video_label_2.setPixmap(pix2)

            # Telemetri
            self.lbl_depth.setText(depth_str)
            self.lbl_lock.setText("AKTİF")
        except KeyboardInterrupt:
            # CTRL+C basılırsa sessizce geç
            pass
        except Exception as e:
            # Qt objeleri silindiğinde veya beklenmedik anda gelen hatalar
            pass

    # ── Başlat / Durdur ─────────────────────────────────────────────
    @Slot()
    def on_start(self):
        if self._running:
            self.append_log("UYARI: Zaten calisiyor")
            return
        self._running = True
        self.lbl_lock.setText("BAŞLATILIYOR...")
        self.video_label.setText(
            f"MOD {self.active_stage} AKTİF\n(ZED bekleniyor...)"
        )
        self.video_label_2.setText(
            f"MOD {self.active_stage} AKTİF\n(Logitech bekleniyor...)"
        )
        
        mode_names = {1: "Manuel", 2: "Sürü", 3: "Otonom"}
        self.append_log(f"▶ {mode_names.get(self.active_stage, 'Bilinmeyen')} mod aktif")

        def on_frames(zed_vis, logi_vis, fps, depth_str):
            self._frame_signal.emit(zed_vis, logi_vis, fps, depth_str)

        def on_log(msg):
            self._log_signal.emit(msg)

        self.worker = DualCameraWorker(on_frames=on_frames, on_log=on_log)
        self.worker.start()

    @Slot()
    def on_stop(self):
        if not self._running:
            return
        self._running = False
        self.append_log("■ Durduruluyor...")
        if self.worker:
            threading.Thread(target=self.worker.stop, daemon=True).start()
            self.worker = None
        self.video_label.setText("ZED\nBEKLENİYOR...")
        self.video_label_2.setText("LOGİTECH\nBEKLENİYOR...")
        self.lbl_depth.setText("—")
        self.lbl_lock.setText("—")

    @Slot(dict)
    def on_pd_changed(self, params):
        self.append_log(f"PD guncellendi: Kp={params['kp']:.3f} Kd={params['kd']:.3f}")

    def closeEvent(self, event):
        self.on_stop()
        event.accept()


# ==================== GİRİŞ NOKTASI ====================

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.showMaximized()
    sys.exit(app.exec())
