class SizeClassifier:
    def __init__(self, large_threshold=10000):
        """
        Args:
            large_threshold: Piksel cinsinden alan eşiği
                           10000 = yaklaşık 100x100 piksel boyutunda obje
                           Video çözünürlüğüne göre ayarlanabilir
        """
        self.large_threshold = large_threshold
    
    def classify(self, bbox):
        """
        Bounding box alanına göre boyut sınıflandırması
        
        Args:
            bbox: [x1, y1, x2, y2] koordinatları
            
        Returns:
            "large" veya "small"
        """
        x1, y1, x2, y2 = bbox
        area = (x2 - x1) * (y2 - y1)
        
        if area >= self.large_threshold:
            return "large"
        else:
            return "small"
    
    def get_area(self, bbox):
        """Debug için alan değerini döndür"""
        x1, y1, x2, y2 = bbox
        return (x2 - x1) * (y2 - y1)
    
    def calibrate_threshold(self, areas_large, areas_small):
        """
        Gerçek test sonrası eşiği otomatik ayarla
        
        Args:
            areas_large: Büyük balonların alan listesi
            areas_small: Küçük balonların alan listesi
        """
        import numpy as np
        avg_large = np.mean(areas_large)
        avg_small = np.mean(areas_small)
        self.large_threshold = (avg_large + avg_small) / 2
        print(f"Yeni eşik değeri: {self.large_threshold:.0f}")
