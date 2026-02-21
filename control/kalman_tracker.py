import cv2
import numpy as np

class TargetKalman:
    def __init__(self, x, y):
        # 4 durum: [x, y, vx, vy]  —  2 ölçüm: [x, y]
        self.kf = cv2.KalmanFilter(4, 2)

        # Başlangıç geçiş matrisi (dt = 1 varsayılan, her predict'te güncelleyeceğiz)
        self.kf.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], np.float32)

        # Ölçüm matrisi: pozisyonu direkt ölçüyoruz
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], np.float32)

        # Süreç ve ölçüm gürültüsü (gerekirse sonra ince ayar yapacağız)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)

        # Başlangıç durumu: konum = ölçüm, hız = 0
        self.kf.statePost = np.array([[x], [y], [0.0], [0.0]], np.float32)

    def predict(self, dt=None):
        # dt geldiyse geçiş matrisini güncelle (sabit hız modeli)
        if dt is not None:
            self.kf.transitionMatrix = np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1,  0],
                [0, 0, 0,  1]
            ], np.float32)

        state_pred = self.kf.predict()
        x, y, vx, vy = state_pred.flatten()
        return (float(x), float(y)), (float(vx), float(vy))

    def correct(self, x_meas, y_meas):
        meas = np.array([[np.float32(x_meas)], [np.float32(y_meas)]])
        state_post = self.kf.correct(meas)
        x, y, vx, vy = state_post.flatten()
        return (float(x), float(y)), (float(vx), float(vy))
