import pyzed.sl as sl
import cv2
import numpy as np
import math
import os
from sahi import AutoDetectionModel
from sahi.predict import get_sliced_prediction

# ─── AYARLAR ───────────────────────────────────────────────
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "balon_baska.pt")

CONF_THRESH   = 0.35                  # Güven eşiği (SAHI ile biraz düşürülebilir)
SLICE_SIZE    = 480                    # Dilim boyutu (piksel)
OVERLAP_RATIO = 0.2                    # Dilimler arası örtüşme oranı
# ───────────────────────────────────────────────────────────

def init_zed():
    """ZED 2i kamerasını derinlik moduyla başlatır."""
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution   = sl.RESOLUTION.HD720   # 720p – iyi hız/kalite dengesi
    init_params.camera_fps          = 60
    init_params.depth_mode          = sl.DEPTH_MODE.ULTRA   # En hassas mod
    init_params.coordinate_units    = sl.UNIT.METER         # Mesafe birimi: METRE
    init_params.depth_maximum_distance = 20.0               # Maks. 20 metre

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED kamera açılamadı: {status}")
    print("✅ ZED 2i başarıyla açıldı.")
    return zed


def get_distance_at_bbox(point_cloud, x1, y1, x2, y2):
    """
    Tespit edilen bounding box'ın merkezindeki piksel için
    ZED point cloud kullanarak Öklid mesafesi hesaplar.
    Daha kararlı sonuç için merkez etrafında 5x5 piksel
    ortalaması alınır.
    """
    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)

    distances = []
    for dx in range(-2, 3):       # 5x5 komşuluk
        for dy in range(-2, 3):
            err, point = point_cloud.get_value(cx + dx, cy + dy)
            if (err == sl.ERROR_CODE.SUCCESS
                    and not math.isnan(point[0])
                    and not math.isinf(point[0])):
                dist = math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
                if 0.1 < dist < 30:   # Geçerli aralık kontrolü
                    distances.append(dist)

    return float(np.median(distances)) if distances else None


def draw_overlay(frame, box, distance, label="Balon"):
    """Tespit kutusunu, mesafeyi ve etiketi çizer."""
    x1, y1, x2, y2 = map(int, box)

    # Bounding box
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 2)

    # Mesafe metni
    if distance is not None:
        text = f"{label}: {distance:.2f} m"
        color = (0, 255, 0) if distance > 1.0 else (0, 0, 255)
    else:
        text = f"{label}: ?"
        color = (0, 0, 255)

    # Metin arka planı (okunabilirlik için)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    cv2.rectangle(frame, (x1, y1 - th - 10), (x1 + tw + 6, y1), (0, 0, 0), -1)
    cv2.putText(frame, text, (x1 + 3, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # Merkez noktası
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    cv2.circle(frame, (cx, cy), 5, (0, 200, 255), -1)


def main():
    zed = init_zed()

    # ─── SAHI + FP16 Model Yükleme ───
    detection_model = AutoDetectionModel.from_pretrained(
        model_type="yolov8",
        model_path=MODEL_PATH,
        confidence_threshold=CONF_THRESH,
        device="cuda:0",          # GPU kullan
    )
    print(f"✅ SAHI modeli yüklendi (FP16): {MODEL_PATH}")

    image       = sl.Mat()
    point_cloud = sl.Mat()
    runtime     = sl.RuntimeParameters()
    runtime.confidence_threshold  = 50
    runtime.texture_confidence_threshold = 100

    print(f"▶  SAHI dilimli çıkarım: {SLICE_SIZE}x{SLICE_SIZE}, örtüşme: {OVERLAP_RATIO}")
    print("▶  Çalışıyor... Çıkmak için 'q' tuşuna basın.")

    while True:
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        # ZED'den sol kamera görüntüsü ve point cloud al
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        # NumPy dizisine dönüştür (BGR)
        frame = image.get_data()[:, :, :3].copy()

        # ─── SAHI Dilimli Çıkarım ───
        result = get_sliced_prediction(
            frame,
            detection_model,
            slice_height=SLICE_SIZE,
            slice_width=SLICE_SIZE,
            overlap_height_ratio=OVERLAP_RATIO,
            overlap_width_ratio=OVERLAP_RATIO,
            verbose=0,
        )

        # Her tespit için mesafeyi hesapla ve çiz
        for pred in result.object_prediction_list:
            cls_name = pred.category.name
            if cls_name.lower() != "person":
                continue  # Sadece insanları işle

            bbox = pred.bbox
            x1, y1, x2, y2 = bbox.minx, bbox.miny, bbox.maxx, bbox.maxy
            distance = get_distance_at_bbox(point_cloud, x1, y1, x2, y2)
            draw_overlay(frame, (x1, y1, x2, y2), distance, label=cls_name)

        # Sol üst köşeye FPS bilgisi
        fps_text = f"FPS: {zed.get_current_fps():.1f}"
        cv2.putText(frame, fps_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow("ZED 2i – SAHI Tespit", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    zed.close()
    print("📷 Kamera kapatıldı.")


if __name__ == "__main__":
    main()
