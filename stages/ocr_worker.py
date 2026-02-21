# stages/ocr_worker.py
import cv2
import numpy as np
import pytesseract
from pytesseract import Output
from PySide6.QtCore import QThread, Signal, Slot


class OcrWorker(QThread):
    result_ready = Signal(str, float)  # letter, conf
    log = Signal(str)

    def __init__(self, parent=None, whitelist="AB", min_conf=60.0):
        super().__init__(parent)
        self._latest_roi = None
        self._running = True
        self.whitelist = whitelist
        self.min_conf = float(min_conf)

    @Slot(object)
    def submit_roi(self, roi_bgr):
        # roi_bgr: numpy array (BGR)
        self._latest_roi = roi_bgr

    def stop(self):
        self._running = False

    def run(self):
        while self._running:
            if self._latest_roi is None:
                self.msleep(10)
                continue

            roi = self._latest_roi
            self._latest_roi = None

            try:
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                gray = cv2.resize(
                    gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR
                )
                gray = cv2.GaussianBlur(gray, (3, 3), 0)
                _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

                config = (
                    rf"--oem 3 --psm 10 -c tessedit_char_whitelist={self.whitelist}"
                )
                data = pytesseract.image_to_data(
                    th, config=config, output_type=Output.DICT
                )

                best_letter, best_conf = None, -1.0
                for t, c in zip(data["text"], data["conf"]):
                    t = (t or "").strip()
                    try:
                        c = float(c)
                    except Exception:
                        continue
                    if t in tuple(self.whitelist) and c > best_conf:
                        best_letter, best_conf = t, c

                if best_letter is not None and best_conf >= self.min_conf:
                    self.result_ready.emit(best_letter, best_conf)

            except Exception as e:
                self.log.emit(f"OCR hata: {e}")
