import time
import cv2
import numpy as np
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from ultralytics import YOLO

from zed_camera import ZedCamera

# ================== MODEL (TensorRT engine) ==================

ENGINE_PATH = r"C:\Users\Tufan\Desktop\yolo8_proje_goruntu\balon_baska.engine"
model = YOLO(ENGINE_PATH)

# ================== KAMERALARI AÇ ==================

zed = ZedCamera(
    resolution="HD720",
    fps=30,
    depth_mode="PERFORMANCE",       # ULTRA → PERFORMANCE (daha hızlı grab)
    max_depth_m=20.0,
)
zed.open()

logi_cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
logi_cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
logi_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
logi_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)       # Buffer = 1 (eski frame birikmesin)

if not logi_cap.isOpened():
    print("Logitech kamera açılamadı! (Index 1)")
    exit(1)

print("İki kamera + TensorRT model hazır, 'q' ile çıkabilirsin.")

# ================== PIPELINE YARDIMCI FONKSİYONLARI ==================

def grab_zed():
    """ZED kamerasından bir frame yakalar (ThreadPoolExecutor içinde çalışır)."""
    return zed.grab_frame()

def grab_logi():
    """Logitech kamerasından bir frame yakalar (ThreadPoolExecutor içinde çalışır)."""
    ret, frame = logi_cap.read()
    if ret:
        return frame
    return None

# ================== FPS HESABI ==================

DISPLAY_W, DISPLAY_H = 640, 480

fps_times = deque(maxlen=30)   # son 30 frame'in zamanını tut
avg_fps = 0.0

# ================== ANA DÖNGÜ (PIPELINE) ==================

executor = ThreadPoolExecutor(max_workers=2)

# İlk frame'leri başlat (pipeline prime)
future_zed  = executor.submit(grab_zed)
future_logi = executor.submit(grab_logi)

while True:
    # ---- Paralel kamera sonuçlarını al ----
    zed_result = future_zed.result()       # (rgb, depth) veya (None, None)
    logi_frame = future_logi.result()      # frame veya None

    # ---- Hemen sonraki frame'leri paralel olarak başlat (pipeline) ----
    future_zed  = executor.submit(grab_zed)
    future_logi = executor.submit(grab_logi)

    rgb_zed, depth_zed = zed_result
    if rgb_zed is None or logi_frame is None:
        continue

    # ---- YOLO batch inference (2 görüntü) ----
    results = model.predict(
        source=[rgb_zed, logi_frame],
        imgsz=480,
        conf=0.35,
        device=0,
        half=True,
        verbose=False,
    )

    res_zed, res_logi = results[0], results[1]

    # ---- ZED sonuçlarını çiz ----
    zed_vis = rgb_zed.copy()
    for box in res_zed.boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        cls_id = int(box.cls[0].cpu().numpy())
        conf  = float(box.conf[0].cpu().numpy())
        label = f"{res_zed.names[cls_id]} {conf:.2f}"
        cv2.rectangle(zed_vis, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
        cv2.putText(zed_vis, label, (int(x1), int(y1)-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    # ---- Logitech sonuçlarını çiz ----
    logi_vis = logi_frame.copy()
    for box in res_logi.boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        cls_id = int(box.cls[0].cpu().numpy())
        conf  = float(box.conf[0].cpu().numpy())
        label = f"{res_logi.names[cls_id]} {conf:.2f}"
        cv2.rectangle(logi_vis, (int(x1), int(y1)), (int(x2), int(y2)), (0,0,255), 2)
        cv2.putText(logi_vis, label, (int(x1), int(y1)-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

    # ---- FPS (son 30 frame kayan ortalaması) ----
    fps_times.append(time.time())
    if len(fps_times) >= 2:
        avg_fps = (len(fps_times) - 1) / (fps_times[-1] - fps_times[0])

    # ---- GÖRÜNTÜLERİ YAN YANA GÖSTER ----
    zed_disp  = cv2.resize(zed_vis,  (DISPLAY_W, DISPLAY_H))
    logi_disp = cv2.resize(logi_vis, (DISPLAY_W, DISPLAY_H))

    concat = cv2.hconcat([zed_disp, logi_disp])

    fps_text = f"FPS (avg): {avg_fps:.1f}"
    cv2.putText(concat, fps_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    cv2.imshow("ZED (sol)  |  Logitech (sag)", concat)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ================== TEMİZLİK ==================
executor.shutdown(wait=False)
logi_cap.release()
zed.close()
cv2.destroyAllWindows()
