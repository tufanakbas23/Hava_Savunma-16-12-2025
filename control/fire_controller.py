"""
control/fire_controller.py

Ortak ateş kontrol mantığı.

- Stage 1: Manuel tetik (kullanıcı butona basınca direkt ateş)
- Stage 2: Otonom – yaklaşma + mesafe + kilit süresi (örn. 4 sn) sonrası ateş
- Stage 3: Aynı arayüz üzerinden genişletilebilir (OCR, LiDAR vb. ek şartlarla)

Bu modül yalnızca karar mantığını tutar; gerçek donanım ateş komutu
Arduino arayüzü (`ArduinoInterface.send(fire=...)`) tarafından yürütülür.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class FireDecision:
    should_fire: bool
    locked: bool
    lock_duration_s: float


class FireController:
    """
    Ateş kontrolü için ortak stateful sınıf.

    - Stage 1: Manuel tetik → doğrudan fire
    - Stage 2: Yaklaşan hedef + mesafe onayı + kilit süresi → fire
    """

    def __init__(self, fire_after_lock_s: float = 4.0):
        # Hedefe kilitlenme süresi (saniye)
        self.fire_after_lock_s = float(fire_after_lock_s)
        self.locked_target_id: Optional[int] = None
        self.lock_acquired_time: Optional[float] = None
        self.current_lock_duration_s: float = 0.0

    # ─────────────────── Genel Yardımcılar ───────────────────

    def reset(self) -> None:
        """Kilitlenme zamanlayıcısını sıfırlar."""
        self.locked_target_id = None
        self.lock_acquired_time = None
        self.current_lock_duration_s = 0.0

    def _update_lock(self, tid: int, locked_now: bool, now: float) -> bool:
        """
        Hedef kilidinin süresini takip eder.

        Returns:
            bool: Şu anda kilit süresi fire_after_lock_s eşiğini aştı mı?
        """
        if locked_now:
            if self.locked_target_id != tid:
                # Yeni hedefe kilit
                self.locked_target_id = tid
                self.lock_acquired_time = now
                self.current_lock_duration_s = 0.0
            else:
                if self.lock_acquired_time is not None:
                    self.current_lock_duration_s = max(
                        0.0, now - self.lock_acquired_time
                    )
            return self.current_lock_duration_s >= self.fire_after_lock_s

        # Kilit kaybedildi
        self.reset()
        return False

    # ─────────────────── Stage 1: Manuel Mod ───────────────────

    def update_stage1(self, manual_trigger: bool) -> FireDecision:
        """
        Stage 1 (manuel) için ateş kararı.

        Args:
            manual_trigger: Kullanıcı tetik butonuna bastı mı?

        Returns:
            FireDecision: should_fire sadece manuel triger'a bağlıdır.
        """
        # Stage 1'de kilit/mesafe zorunlu değil, doğrudan kullanıcıya bırakılır.
        self.reset()
        return FireDecision(
            should_fire=bool(manual_trigger),
            locked=False,
            lock_duration_s=0.0,
        )

    # ─────────────────── Stage 2: Otonom Mod ───────────────────

    def update_stage2(
        self,
        tid: Optional[int],
        locked_now: bool,
        depth_m: Optional[float],
        vz: float,
        have_target: bool,
        now: Optional[float] = None,
        engage_ok: bool = False,
    ) -> FireDecision:
        """
        Stage 2 için ateş kararı.

        Args:
            tid: Takip edilen hedef ID'si (None ise hedef yok)
            locked_now: Bu frame'de IBVS kilidi var mı?
            depth_m: Hedefin derinliği (metre), None olabilir
            vz: Hedefin yaklaşma hızı (m/s, pozitif = yaklaşıyor)
            have_target: En az bir hedef var mı?
            now: Zaman damgası (saniye). None ise time.time() kullanılır.
            engage_ok: Mesafe + yaklaşma kuralları sağlandı mı?

        Returns:
            FireDecision: should_fire True ise ateş tetiklenmeli.
        """
        if now is None:
            now = time.time()

        if not have_target or tid is None or depth_m is None:
            self.reset()
            return FireDecision(
                should_fire=False,
                locked=False,
                lock_duration_s=self.current_lock_duration_s,
            )

        # Kilit süresi ≥ fire_after_lock_s mi?
        locked_for_long = self._update_lock(tid, locked_now, now)

        # Nihai ateş kararı: sistem angajmana hazır mı (engage_ok)
        # VE kilit süresi eşiği aşıldı mı?
        should_fire = bool(engage_ok and locked_for_long)

        return FireDecision(
            should_fire=should_fire,
            locked=locked_now,
            lock_duration_s=self.current_lock_duration_s,
        )

