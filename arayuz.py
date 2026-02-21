# ============================================================================
# DOSYA İÇERİĞİ / SATIR NUMARALARI
# ============================================================================
#
# [SATIR 64-101]    İMPORT'LAR & ORTAM AYARLARI
#                   - OpenCV video ayarları
#                   - PySide6 widget import'ları
#                   - Hardware/Stage import'ları
#
# [SATIR 104-243]   WORKER THREAD'LER (QThread)
#                   - Stage1Worker (Satır 108-155)
#                   - Stage2Worker (Satır 158-193)
#                   - Stage3Worker (Satır 196-243)
#
# [SATIR 246-434]   RENK KALİBRASYON WİDGET'I
#                   - ColorCalibrationWidget class
#                   - Video tıklama ile LAB renk örnekleme
#                   - JSON kaydetme/yükleme
#
# [SATIR 439-478]   PD KONTROLÖR AYARLARI WİDGET'İ
#                   - PDTuningWidget class
#                   - Kp/Kd parametre ayarlama
#
# [SATIR 483-998]   ANA PENCERE (MainWindow)
#
#   [483-498]       __init__ - Başlatma
#   [500-568]       init_ui - Ana layout oluşturma
#   [570-633]       create_control_tab - Kontrol tab'ı
#   [635-653]       create_stage1_controls - Stage 1 butonları
#   [655-669]       create_stage2_controls - Stage 2 butonları
#   [671-695]       create_stage3_controls - Stage 3 butonları
#   [697-703]       create_calibration_tab - Kalibrasyon tab'ı
#   [705-711]       create_pd_tab - PD ayarları tab'ı
#   [713-791]       apply_styles - CSS stilleri
#   [793-797]       setup_shortcuts - Klavye kısayolları (S/F/N/Enter)
#   [799-805]       on_tab_changed - Tab değişim event'i
#   [807-825]       on_video_click - Video tıklama (kalibrasyon)
#   [827-834]       append_log - Log mesajları
#   [836-852]       on_stage_changed - Stage seçim event'i
#   [854-879]       on_start - Sistem başlatma
#   [881-888]       on_stop - Sistem durdurma
#   [890-903]       on_estop - Acil durdur
#   [905-915]       on_track/on_fire/on_next - Stage 1 kontrolleri
#   [917-921]       on_auto - Stage 2 otonom mod
#   [923-927]       on_stage3_cycle - Stage 3 döngü başlat
#   [929-937]       on_pd_changed - PD parametre güncelleme
#   [939-949]       on_frame - Video frame görüntüleme
#   [951-984]       on_telemetry - Telemetri verileri güncelleme
#   [986-993]       closeEvent - Pencere kapanış event'i
#
# [SATIR 996-1002]  MAIN - Uygulama başlatma
#
# ============================================================================
# HIZLI ERİŞİM İÇİN ÖNEMLİ SATIRLAR:
# ============================================================================
# Video boyutu ayarı:        Satır 528 (setFixedSize)
# Log yüksekliği ayarı:      Satır 559 (setFixedHeight)
# Stage kontrol yüksekliği:  Satır 609-610 (setMinimumHeight/setMaximumHeight)
# CSS stilleri:              Satır 713-791
# Model path'leri:           Satır 864, 867, 870
# Arduino port ayarı:        Satır 859
# ============================================================================


import os

os.environ["OPENCV_VIDEOIO_PRIORITY_MSMF"] = "0"
os.environ["OPENCV_VIDEOIO_PRIORITY_DSHOW"] = "100"

import sys
import time
import cv2
import json
import numpy as np
from config.system_config import SystemConfig
from PySide6.QtCore import Qt, QThread, Signal, Slot, QSize
from PySide6.QtGui import QImage, QPixmap, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLabel,
    QPushButton,
    QComboBox,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QGridLayout,
    QPlainTextEdit,
    QTabWidget,
    QDoubleSpinBox,
    QScrollArea,
)

from hardware.arduino_interface import ArduinoInterface
from stages.stage1 import Stage1Engine
from stages.stage2 import Stage2Engine
from stages.stage3 import Stage3Engine
from stages.ocr_worker import OcrWorker


# ==================== WORKER THREADS ====================


class Stage1Worker(QThread):
    frame_ready = Signal(QImage)
    telemetry = Signal(dict)
    log = Signal(str)

    def __init__(self, model_path, arduino, camera_index=0, parent=None):
        super().__init__(parent)
        self.engine = Stage1Engine(model_path, arduino, camera_index)

    def stop(self):
        self.engine.should_stop = True

    def set_tracking(self, enabled: bool):
        self.engine.tracking_enabled = enabled
        if enabled:
            self.engine.cycle_armed = True
            self.engine.fire_requested = False

    def request_fire(self):
        self.engine.fire_requested = True

    def reset_cycle(self):
        self.engine.tracking_enabled = True
        self.engine.fire_requested = False
        self.engine.cycle_armed = True

    def run(self):
        self.log.emit("Stage1 başlatıldı")
        try:
            for frame, telem in self.engine.run():
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
                self.frame_ready.emit(qimg)
                self.telemetry.emit(telem)
        except Exception as e:
            self.log.emit(f"HATA: {e}")
        finally:
            self.log.emit("Stage1 durduruldu")


class Stage2Worker(QThread):
    frame_ready = Signal(QImage)
    telemetry = Signal(dict)
    log = Signal(str)

    def __init__(self, model_path, arduino, camera_index=0, parent=None):
        super().__init__(parent)
        self.engine = Stage2Engine(model_path, arduino, camera_index)

    def stop(self):
        self.engine.should_stop = True

    def set_auto(self, enabled: bool):
        self.engine.auto_tracking_enabled = enabled

    def run(self):
        self.log.emit("Stage2 başlatıldı (Otonom)")
        try:
            for frame, telem in self.engine.run():
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
                self.frame_ready.emit(qimg)
                self.telemetry.emit(telem)
        except Exception as e:
            self.log.emit(f"HATA: {e}")
        finally:
            self.log.emit("Stage2 durduruldu")


class Stage3Worker(QThread):
    frame_ready = Signal(QImage)
    telemetry = Signal(dict)
    log = Signal(str)
    ocr_request = Signal(object)

    def __init__(self, model_path, arduino, camera_index=0, parent=None):
        super().__init__(parent)
        self.engine = Stage3Engine(
            model_path, arduino=arduino, camera_index=camera_index
        )
        self.ocr_worker = OcrWorker(whitelist="AB", min_conf=60.0)
        self.ocr_worker.result_ready.connect(self.on_ocr_result)
        self.ocr_worker.log.connect(lambda m: self.log.emit(m))
        self.ocr_request.connect(self.ocr_worker.submit_roi)

    def stop(self):
        self.engine.should_stop = True
        self.ocr_worker.stop()

    def start_cycle(self):
        self.engine.start_cycle()

    @Slot(str, float)
    def on_ocr_result(self, letter: str, conf: float):
        self.engine.submit_ocr_result(letter, conf)
        self.log.emit(f"OCR: {letter} (conf={conf:.1f}%)")

    def run(self):
        self.log.emit("Stage3 başlatıldı")
        self.ocr_worker.start()
        try:
            for frame, telem in self.engine.run():
                roi = telem.pop("ocr_roi", None)
                if roi is not None:
                    self.ocr_request.emit(roi)

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
                self.frame_ready.emit(qimg)
                self.telemetry.emit(telem)
        except Exception as e:
            self.log.emit(f"HATA: {e}")
        finally:
            self.ocr_worker.stop()
            self.ocr_worker.wait(2000)
            self.log.emit("Stage3 durduruldu")


# ==================== RENK KALİBRASYON WİDGET ====================


class ColorCalibrationWidget(QWidget):
    log_signal = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.clicked_values = []
        self.current_color = "red"
        self.calibrations = {
            "red": {"L_center": 60, "A_center": 30, "B_center": 20, "radius_ab": 15},
            "blue": {"L_center": 50, "A_center": -20, "B_center": -25, "radius_ab": 15},
            "green": {"L_center": 60, "A_center": -30, "B_center": 30, "radius_ab": 15},
        }
        self.init_ui()
        self.load_calibration()

    def init_ui(self):
        layout = QVBoxLayout(self)

        info = QLabel("Stage durdukken video üzerine 10 kez tıklayın")
        info.setStyleSheet("color: #888; font-weight: bold; font-size: 11pt;")
        layout.addWidget(info)

        ctrl_layout = QHBoxLayout()
        ctrl_layout.addWidget(QLabel("Renk:"))
        self.color_combo = QComboBox()
        self.color_combo.addItems(["red", "blue", "green"])
        self.color_combo.currentTextChanged.connect(self.on_color_changed)
        ctrl_layout.addWidget(self.color_combo)

        self.lbl_samples = QLabel("Örnek: 0/10")
        ctrl_layout.addWidget(self.lbl_samples)

        btn_clear = QPushButton("Temizle")
        btn_clear.clicked.connect(self.clear_samples)
        ctrl_layout.addWidget(btn_clear)
        ctrl_layout.addStretch()
        layout.addLayout(ctrl_layout)

        calc_box = QGroupBox("Hesaplanan Değerler")
        calc_layout = QGridLayout(calc_box)
        self.lbl_l = QLabel("L: -")
        self.lbl_a = QLabel("A: -")
        self.lbl_b = QLabel("B: -")
        self.lbl_r = QLabel("Radius: -")
        calc_layout.addWidget(self.lbl_l, 0, 0)
        calc_layout.addWidget(self.lbl_a, 1, 0)
        calc_layout.addWidget(self.lbl_b, 2, 0)
        calc_layout.addWidget(self.lbl_r, 3, 0)
        layout.addWidget(calc_box)

        btn_layout = QHBoxLayout()
        btn_save = QPushButton("Kaydet")
        btn_load = QPushButton("Yükle")
        btn_export = QPushButton("Kod Çıkar")
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

        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        L, A, B = lab[y, x]
        self.clicked_values.append([float(L), float(A), float(B)])
        self.lbl_samples.setText(f"Örnek: {len(self.clicked_values)}/10")

        if len(self.clicked_values) == 10:
            self.auto_calculate()

    def auto_calculate(self):
        if len(self.clicked_values) < 3:
            return

        vals = np.array(self.clicked_values)
        L_center = float(np.mean(vals[:, 0]))
        A_center = float(np.mean(vals[:, 1]))
        B_center = float(np.mean(vals[:, 2]))
        dists_ab = np.sqrt((vals[:, 1] - A_center) ** 2 + (vals[:, 2] - B_center) ** 2)
        radius_ab = float(np.max(dists_ab) + 5)

        self.calibrations[self.current_color] = {
            "L_center": L_center,
            "A_center": A_center,
            "B_center": B_center,
            "radius_ab": radius_ab,
        }
        self.update_display()
        self.log_signal.emit(f"{self.current_color.upper()} kalibrasyon tamamlandı")

    def update_display(self):
        cal = self.calibrations[self.current_color]
        self.lbl_l.setText(f"L: {cal['L_center']:.1f}")
        self.lbl_a.setText(f"A: {cal['A_center']:.1f}")
        self.lbl_b.setText(f"B: {cal['B_center']:.1f}")
        self.lbl_r.setText(f"Radius: {cal['radius_ab']:.1f}")

    def on_color_changed(self, color):
        self.current_color = color
        self.clicked_values = []
        self.lbl_samples.setText("Örnek: 0/10")
        self.update_display()

    def clear_samples(self):
        self.clicked_values = []
        self.lbl_samples.setText("Örnek: 0/10")

    def save_calibration(self):
        with open("color_calibration.json", "w") as f:
            json.dump(self.calibrations, f, indent=2)
        self.log_signal.emit("color_calibration.json kaydedildi")

    def load_calibration(self):
        try:
            with open("color_calibration.json", "r") as f:
                self.calibrations = json.load(f)
            self.update_display()
            self.log_signal.emit("color_calibration.json yüklendi")
        except FileNotFoundError:
            pass

    def export_code(self):
        cal = self.calibrations[self.current_color]
        print("\n" + "=" * 60)
        print(f"KALIBRASYON KODU - {self.current_color.upper()}")
        print("=" * 60)
        print(
            f"self.{self.current_color}_center = np.array([{cal['A_center']:.1f}, {cal['B_center']:.1f}])"
        )
        print(f"self.max_dist_{self.current_color} = {cal['radius_ab']:.1f}")
        print("=" * 60 + "\n")
        self.log_signal.emit(f"{self.current_color.upper()} kodu konsola yazdırıldı")


# ==================== PD AYARLARI WİDGET ====================


class PDTuningWidget(QWidget):
    params_changed = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)

        info = QLabel("PD Kontrolör Parametreleri")
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
        kd_layout.addWidget(QLabel("Kd (Türev):"))
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
        params = {
            "kp": self.spin_kp.value(),
            "kd": self.spin_kd.value(),
        }
        self.params_changed.emit(params)


# ==================== ANA PENCERE ====================


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hava Savunma Sistemi")
        self.resize(1700, 950)

        self.worker = None
        self.arduino = None
        self.active_stage = 1
        self.calibration_mode = False
        self.current_frame_bgr = None

        self.init_ui()
        self.setup_shortcuts()
        self.apply_styles()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # ===== SOL PANEL: VİDEO (SABİT BOYUT) =====
        left_panel = QVBoxLayout()
        left_panel.setSpacing(0)

        self.video_label = QLabel("KAMERA BEKLENIYOR...")
        self.video_label.setAlignment(Qt.AlignCenter)
        # ✅ SABİT BOYUT - artık kaymayacak
        self.video_label.setFixedSize(1050, 750)
        self.video_label.setStyleSheet(
            "background:#000; color:#0f0; border:2px solid #333; "
            "font-size:14pt; font-family:Consolas;"
        )
        self.video_label.mousePressEvent = self.on_video_click
        left_panel.addWidget(self.video_label)

        main_layout.addLayout(left_panel)

        # ===== SAĞ PANEL: KONTROLLER =====
        right_panel = QVBoxLayout()
        right_panel.setSpacing(5)

        # Tab Widget
        self.tab_widget = QTabWidget()
        self.tab_widget.setMinimumWidth(550)
        self.tab_widget.setMaximumWidth(600)
        self.tab_widget.currentChanged.connect(self.on_tab_changed)

        # TAB 1: KONTROL
        self.control_tab = self.create_control_tab()
        self.tab_widget.addTab(self.control_tab, "Kontrol")

        # TAB 2: KALİBRASYON
        self.calibration_tab = self.create_calibration_tab()
        self.tab_widget.addTab(self.calibration_tab, "Kalibrasyon")

        # TAB 3: PD AYARLARI
        self.pd_tab = self.create_pd_tab()
        self.tab_widget.addTab(self.pd_tab, "PD Ayarlari")

        right_panel.addWidget(self.tab_widget)

        # LOG - ✅ DAHA FAZLA YÜKSEKLİK
        log_box = QGroupBox("Log")
        log_layout = QVBoxLayout(log_box)
        log_layout.setContentsMargins(5, 5, 5, 5)

        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(1000)
        # ✅ Sabit yükseklik - artık kaymayacak
        self.log_box.setFixedHeight(300)
        self.log_box.setStyleSheet(
            "background:#000; color:#0f0; font-family:Consolas; font-size:9pt;"
        )
        log_layout.addWidget(self.log_box)
        right_panel.addWidget(log_box)

        main_layout.addLayout(right_panel)

    def create_control_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(8)

        # SİSTEM KONTROLÜ
        sys_box = QGroupBox("Sistem Kontrolu")
        sys_layout = QGridLayout(sys_box)

        sys_layout.addWidget(QLabel("Stage:"), 0, 0)
        self.stage_combo = QComboBox()
        self.stage_combo.addItems(
            ["Stage 1 - Hareketli Hedef", "Stage 2 - Dost/Dusman", "Stage 3 - Angajman"]
        )
        self.stage_combo.currentIndexChanged.connect(self.on_stage_changed)
        sys_layout.addWidget(self.stage_combo, 0, 1, 1, 2)

        self.btn_start = QPushButton("START")
        self.btn_stop = QPushButton("STOP")
        self.btn_estop = QPushButton("E-STOP")

        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_estop.clicked.connect(self.on_estop)

        sys_layout.addWidget(self.btn_start, 1, 0)
        sys_layout.addWidget(self.btn_stop, 1, 1)
        sys_layout.addWidget(self.btn_estop, 1, 2)

        layout.addWidget(sys_box)

        # TELEMETRİ
        telem_box = QGroupBox("Telemetri")
        telem_layout = QGridLayout(telem_box)

        self.lbl_stage = QLabel("Stage: 1")
        self.lbl_fps = QLabel("FPS: -")
        self.lbl_lock = QLabel("LOCK: -")
        self.lbl_target = QLabel("Target: -")

        for i, lbl in enumerate(
            [self.lbl_stage, self.lbl_fps, self.lbl_lock, self.lbl_target]
        ):
            lbl.setStyleSheet("font-size:10pt; font-weight:bold;")
            telem_layout.addWidget(lbl, i, 0)

        layout.addWidget(telem_box)

        # ✅ STAGE KONTROL ALANI - SCROLL AREA İLE
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setMinimumHeight(230)
        scroll_area.setMaximumHeight(230)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        stage_control_container = QWidget()
        stage_control_layout = QVBoxLayout(stage_control_container)
        stage_control_layout.setContentsMargins(0, 0, 0, 0)

        self.stage1_box = self.create_stage1_controls()
        self.stage2_box = self.create_stage2_controls()
        self.stage3_box = self.create_stage3_controls()

        stage_control_layout.addWidget(self.stage1_box)
        stage_control_layout.addWidget(self.stage2_box)
        stage_control_layout.addWidget(self.stage3_box)
        stage_control_layout.addStretch()

        self.stage2_box.hide()
        self.stage3_box.hide()

        scroll_area.setWidget(stage_control_container)
        layout.addWidget(scroll_area)

        layout.addStretch()
        return tab

    def create_stage1_controls(self):
        box = QGroupBox("Stage 1 - Kontrol")
        layout = QGridLayout(box)

        # Mod seçimi
        self.btn_manual = QPushButton("MANUEL")
        self.btn_autonomous = QPushButton("OTONOM")
        self.btn_manual.setCheckable(True)
        self.btn_autonomous.setCheckable(True)
        self.btn_manual.setChecked(True)  # Varsayılan manuel
        
        self.btn_manual.clicked.connect(self.on_manual_mode)
        self.btn_autonomous.clicked.connect(self.on_autonomous_mode)
        
        layout.addWidget(self.btn_manual, 0, 0)
        layout.addWidget(self.btn_autonomous, 0, 1)

        # Tracking kontrolleri (sadece otonom modda aktif)
        self.btn_track = QPushButton("Track (S)")
        self.btn_fire = QPushButton("Fire (F)")
        self.btn_next = QPushButton("Next (N)")

        self.btn_track.clicked.connect(self.on_track)
        self.btn_fire.clicked.connect(self.on_fire)
        self.btn_next.clicked.connect(self.on_next)

        layout.addWidget(self.btn_track, 1, 0)
        layout.addWidget(self.btn_fire, 1, 1)
        layout.addWidget(self.btn_next, 2, 0, 1, 2)
        
        # Mod durumu etiketi
        self.lbl_mode = QLabel("Mod: MANUEL (Joystick aktif)")
        self.lbl_mode.setStyleSheet("font-size:9pt; color:#888;")
        layout.addWidget(self.lbl_mode, 3, 0, 1, 2)

        return box

    def create_stage2_controls(self):
        box = QGroupBox("Stage 2 - Otonom Mod")
        layout = QGridLayout(box)

        self.btn_auto_start = QPushButton("AUTO Start")
        self.btn_auto_stop = QPushButton("AUTO Stop")

        self.btn_auto_start.clicked.connect(lambda: self.on_auto(True))
        self.btn_auto_stop.clicked.connect(lambda: self.on_auto(False))

        layout.addWidget(self.btn_auto_start, 0, 0)
        layout.addWidget(self.btn_auto_stop, 0, 1)

        return box

    def create_stage3_controls(self):
        box = QGroupBox("Stage 3 - Angajman Sistemi")
        layout = QVBoxLayout(box)

        self.btn_s3_cycle = QPushButton("DONGU BASLAT")
        self.btn_s3_cycle.setMinimumHeight(50)
        self.btn_s3_cycle.clicked.connect(self.on_stage3_cycle)
        layout.addWidget(self.btn_s3_cycle)

        self.lbl_s3_state = QLabel("State: IDLE")
        self.lbl_s3_state.setStyleSheet("font-size:11pt; font-weight:bold;")
        layout.addWidget(self.lbl_s3_state)

        self.lbl_s3_info = QLabel("Letter: - | Shape: - | Color: -")
        self.lbl_s3_info.setStyleSheet("font-size:9pt;")
        layout.addWidget(self.lbl_s3_info)

        info_text = QLabel(
            "1. Sistemi angajman tahtasina manuel cevirin\n"
            "2. DONGU BASLAT'a basin (ENTER)\n"
            "3. Sistem otomatik isle yapar\n"
            "4. Bitince manuel tahtaya donun"
        )
        info_text.setStyleSheet("font-size:8pt; color:#666;")
        info_text.setWordWrap(True)
        layout.addWidget(info_text)

        return box

    def create_calibration_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        self.color_calib_widget = ColorCalibrationWidget(parent=self)
        self.color_calib_widget.log_signal.connect(self.append_log)
        layout.addWidget(self.color_calib_widget)
        return tab

    def create_pd_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        self.pd_widget = PDTuningWidget(parent=self)
        self.pd_widget.params_changed.connect(self.on_pd_changed)
        layout.addWidget(self.pd_widget)
        return tab

    def apply_styles(self):
        self.setStyleSheet(
            """
            QMainWindow {
                background-color: #1e1e1e;
            }
            QWidget {
                background-color: #1e1e1e;
                color: #d4d4d4;
                font-family: 'Segoe UI', Arial;
                font-size: 10pt;
            }
            QGroupBox {
                border: 1px solid #3e3e3e;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 8px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                background-color: #2d2d30;
                border: 1px solid #3e3e3e;
                border-radius: 3px;
                padding: 8px 16px;
                color: #d4d4d4;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #3e3e42;
                border: 1px solid #505050;
            }
            QPushButton:pressed {
                background-color: #1e1e1e;
            }
            QPushButton:disabled {
                background-color: #252526;
                color: #656565;
            }
            QComboBox {
                background-color: #2d2d30;
                border: 1px solid #3e3e3e;
                border-radius: 3px;
                padding: 5px;
            }
            QComboBox:hover {
                border: 1px solid #505050;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox QAbstractItemView {
                background-color: #2d2d30;
                selection-background-color: #3e3e42;
            }
            QLabel {
                color: #d4d4d4;
            }
            QTabWidget::pane {
                border: 1px solid #3e3e3e;
                border-radius: 3px;
            }
            QTabBar::tab {
                background-color: #2d2d30;
                border: 1px solid #3e3e3e;
                padding: 8px 20px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background-color: #1e1e1e;
                border-bottom: 2px solid #0e639c;
            }
            QTabBar::tab:hover {
                background-color: #3e3e42;
            }
            QSpinBox, QDoubleSpinBox {
                background-color: #2d2d30;
                border: 1px solid #3e3e3e;
                border-radius: 3px;
                padding: 4px;
            }
        """
        )

    def setup_shortcuts(self):
        QShortcut(QKeySequence("S"), self, self.on_track)
        QShortcut(QKeySequence("F"), self, self.on_fire)
        QShortcut(QKeySequence("N"), self, self.on_next)
        QShortcut(QKeySequence("Return"), self, self.on_stage3_cycle)

    def on_tab_changed(self, index):
        self.calibration_mode = index == 1
        if hasattr(self, "log_box"):
            if self.calibration_mode:
                self.append_log("Kalibrasyon modu ACIK - video uzerine tiklayin")
            else:
                self.append_log("Kalibrasyon modu KAPALI")

    def on_video_click(self, event):
        if not self.calibration_mode:
            return

        if self.worker and self.worker.isRunning():
            self.append_log("UYARI: Stage calisirken kalibrasyon yapilamaz")
            return

        if self.current_frame_bgr is None:
            self.append_log("UYARI: Henuz frame yok, once START yapin")
            return

        label_w = self.video_label.width()
        label_h = self.video_label.height()
        frame_h, frame_w = self.current_frame_bgr.shape[:2]
        scale_x = frame_w / label_w
        scale_y = frame_h / label_h
        x = int(event.x() * scale_x)
        y = int(event.y() * scale_y)

        self.color_calib_widget.handle_click(x, y, self.current_frame_bgr)

    def append_log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_box.appendPlainText(f"[{ts}] {msg}")
        # ✅ Otomatik scroll en alta
        self.log_box.verticalScrollBar().setValue(
            self.log_box.verticalScrollBar().maximum()
        )

    @Slot(int)
    def on_stage_changed(self, idx: int):
        if self.worker and self.worker.isRunning():
            self.append_log("UYARI: Once STOP yapin, sonra stage degistirin")
            self.stage_combo.blockSignals(True)
            self.stage_combo.setCurrentIndex(self.active_stage - 1)
            self.stage_combo.blockSignals(False)
            return

        self.active_stage = idx + 1
        self.lbl_stage.setText(f"Stage: {self.active_stage}")

        self.stage1_box.setVisible(self.active_stage == 1)
        self.stage2_box.setVisible(self.active_stage == 2)
        self.stage3_box.setVisible(self.active_stage == 3)

        # ✅ BU SATIRI EKLE
        if self.arduino:
            self.arduino.send_stage_led(self.active_stage)

        self.append_log(f"Stage {self.active_stage} secildi")

    @Slot()
    def on_start(self):
        if self.worker and self.worker.isRunning():
            self.append_log("UYARI: Zaten calisiyor")
            return

        # Arduino bağlantısı
        self.arduino = ArduinoInterface(
            port=SystemConfig.ARDUINO_PORT, baudrate=SystemConfig.ARDUINO_BAUDRATE
        )

        # ✅ YENİ: Config'den model path al
        if self.active_stage == 1:
            model_path = SystemConfig.YOLO_MODEL_STAGE1
            self.worker = Stage1Worker(model_path, self.arduino, camera_index=0)
        elif self.active_stage == 2:
            model_path = SystemConfig.YOLO_MODEL_STAGE2
            self.worker = Stage2Worker(model_path, self.arduino, camera_index=0)
        elif self.active_stage == 3:
            model_path = SystemConfig.YOLO_MODEL_STAGE3
            self.worker = Stage3Worker(model_path, self.arduino, camera_index=0)

        self.worker.frame_ready.connect(self.on_frame)
        self.worker.telemetry.connect(self.on_telemetry)
        self.worker.log.connect(self.append_log)
        self.worker.start()

        # Stage LED'i yak
        if self.arduino:
            self.arduino.send_stage_led(self.active_stage)
            # ✅ Arduino'yu otonom moda al (varsayılan: manuel)
            self.arduino.set_mode(autonomous=True)

        self.append_log(f"Stage {self.active_stage} BASLATILDI - Model: {model_path}")

    @Slot()
    def on_stop(self):
        if self.worker:
            self.append_log("STOP istendi...")
            self.worker.stop()
            self.worker.wait(3000)
            self.worker = None

            # ✅ Stage durdu, LED'i söndür
            if self.arduino:
                self.arduino.send_stage_led(0)

            self.append_log("Durduruldu")

    @Slot()
    def on_estop(self):
        self.append_log("ACIL DURDUR!")
        if self.worker:
            if self.active_stage == 1:
                self.worker.engine.tracking_enabled = False
                self.worker.engine.fire_requested = False
            elif self.active_stage == 2:
                self.worker.engine.auto_tracking_enabled = False
            elif self.active_stage == 3:
                self.worker.engine.state = "IDLE"

        if self.arduino:
            self.arduino.send(fire=False)
            # ✅ Acil durum, LED'i söndür
            self.arduino.send_stage_led(0)

    @Slot()
    def on_track(self):
        if self.worker and self.active_stage == 1:
            self.worker.set_tracking(True)
            self.append_log("Takip baslatildi")

    @Slot()
    def on_fire(self):
        if self.worker and self.active_stage == 1:
            self.worker.request_fire()
            self.append_log("Ates istendi")

    @Slot()
    def on_next(self):
        if self.worker and self.active_stage == 1:
            self.worker.reset_cycle()
            self.append_log("Cycle reset")

    @Slot()
    def on_manual_mode(self):
        """Manuel moda geç - Joystick kontrolü aktif"""
        self.btn_manual.setChecked(True)
        self.btn_autonomous.setChecked(False)
        
        if self.arduino:
            self.arduino.set_mode(autonomous=False)
        
        # Tracking'i kapat
        if self.worker and self.active_stage == 1:
            self.worker.set_tracking(False)
        
        self.lbl_mode.setText("Mod: MANUEL (Joystick aktif)")
        self.lbl_mode.setStyleSheet("font-size:9pt; color:#00ff00;")
        self.append_log("MANUEL mod - Joystick kontrolü aktif")

    @Slot()
    def on_autonomous_mode(self):
        """Otonom moda geç - Python/IBVS kontrolü aktif"""
        self.btn_manual.setChecked(False)
        self.btn_autonomous.setChecked(True)
        
        if self.arduino:
            self.arduino.set_mode(autonomous=True)
        
        self.lbl_mode.setText("Mod: OTONOM (Python kontrolü)")
        self.lbl_mode.setStyleSheet("font-size:9pt; color:#00aaff;")
        self.append_log("OTONOM mod - Python kontrolü aktif. Track butonuna basın.")

    @Slot(bool)
    def on_auto(self, enabled: bool):
        if self.worker and self.active_stage == 2:
            self.worker.set_auto(enabled)
            self.append_log(f"AUTO mod: {'ACIK' if enabled else 'KAPALI'}")

    @Slot()
    def on_stage3_cycle(self):
        if self.worker and self.active_stage == 3:
            self.worker.start_cycle()
            self.append_log("Stage3 dongu baslatildi")

    @Slot(dict)
    def on_pd_changed(self, params):
        if self.worker and hasattr(self.worker.engine, "pan_pd"):
            self.worker.engine.pan_pd.kp = params["kp"]
            self.worker.engine.pan_pd.kd = params["kd"]
            self.worker.engine.tilt_pd.kp = params["kp"]
            self.worker.engine.tilt_pd.kd = params["kd"]
            self.append_log(
                f"PD guncellendi: Kp={params['kp']:.3f}, Kd={params['kd']:.3f}"
            )

    @Slot(QImage)
    def on_frame(self, img: QImage):
        if self.worker:
            ptr = img.bits()
            h, w = img.height(), img.width()
            arr = np.array(ptr).reshape(h, w, 3)
            self.current_frame_bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

        pix = QPixmap.fromImage(img)
        self.video_label.setPixmap(
            pix.scaled(
                self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        )

    @Slot(dict)
    def on_telemetry(self, t: dict):
        self.lbl_fps.setText(f"FPS: {t.get('fps', 0):.0f}")

        locked = t.get("locked", None)
        lock_text = "YES" if locked else "NO" if locked is not None else "-"
        self.lbl_lock.setText(f"LOCK: {lock_text}")

        if self.active_stage == 1:
            target_str = (
                f"ID:{t.get('target_id')} {t.get('target_size')}"
                if t.get("have_target")
                else "None"
            )
            self.lbl_target.setText(f"Target: {target_str}")

        elif self.active_stage == 2:
            target_str = (
                f"ID:{t.get('target_id')} {t.get('target_size')} [{t.get('target_color')}]"
                if t.get("have_target")
                else "None"
            )
            self.lbl_target.setText(f"Target: {target_str}")

        elif self.active_stage == 3:
            state = t.get("state", "IDLE")
            self.lbl_s3_state.setText(f"State: {state}")

            self.lbl_s3_info.setText(
                f"Letter: {t.get('letter', '-')} | "
                f"Shape: {t.get('shape', '-')} | "
                f"Color: {t.get('color', '-')}"
            )
            self.lbl_target.setText(f"Target: {t.get('target_id', '-')}")

    def closeEvent(self, event):
        try:
            if self.worker and self.worker.isRunning():
                self.worker.stop()
                self.worker.wait(2000)
        except Exception:
            pass
        
        # Arduino bağlantısını kapat
        try:
            if self.arduino:
                self.arduino.close()
        except Exception:
            pass
        
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    w = MainWindow()
    w.show()
    sys.exit(app.exec())
