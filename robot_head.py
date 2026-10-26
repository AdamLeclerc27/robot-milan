"""
Contr??leur de la T??te et des Paupi??res de Milan 2.
G??re les servomoteurs sur la carte I2C PCA9685 :
- Canal 6 : T??te Pan (Rotation gauche / droite : -70?? ?? +70??, neutre 90??)
- Canal 7 : T??te Tilt (Inclinaison haut / bas : -30?? vers le haut, +45?? vers le sol, neutre 90??)
- Canal 8 : Paupi??re Gauche (0??=ouverte, 60??=ferm??e)
- Canal 9 : Paupi??re Droite (0??=ouverte, 60??=ferm??e)

Offre des animations expressives pour Milan : clignement d'yeux, clin d'??il, r??veil, dodo.
"""

import time
import math
import random
from typing import Optional

try:
    from adafruit_servokit import ServoKit
    HARDWARE_AVAILABLE = True
except (ImportError, Exception):
    HARDWARE_AVAILABLE = False


# --- Limites logicielles de la tete ---
# Tilt (canal 6) : butee basse reelle validee a 30 degres (regard vers le sol). Au-dela de 135, le servo bute.
TILT_MIN_SERVO = 30.0     # servo le plus bas (regard le plus pres du sol)
TILT_MAX_SERVO = 135.0
# Pan (canal 7) : bornes de securite pour ne pas forcer contre les butees
PAN_MIN_SERVO = 45.0
PAN_MAX_SERVO = 135.0
# Vitesse limitee de la tete (pan et tilt), en degres par seconde : mouvement lent et doux
HEAD_MAX_DEG_PER_S = 60.0
HEAD_MAX_MOVE_S = 4.0     # duree maximale d'un mouvement de tete


class RobotHead:
    def __init__(self, kit=None):
        if kit is not None:
            self.kit = kit
        elif HARDWARE_AVAILABLE:
            try:
                self.kit = ServoKit(channels=16)
            except Exception:
                self.kit = None
        else:
            self.kit = None

        # Angles neutres physiques v??rifi??s
        self.pan_angle = 90.0       # Canal 7 : 90?? = Centre droit devant
        self.tilt_angle = 88.0      # Canal 6 : 88?? = Regard horizontal droit devant
        self.eyelid_left = 0.0      # Canal 8 : 0?? = Ouverte
        self.eyelid_right = 180.0   # Canal 9 : 180?? = Ouverte

    def apply_hardware(self):
        """Applique les angles aux servos physiques si connect??s."""
        if self.kit:
            try:
                # Canal 7 = Pan (rotation gauche/droite)
                self.kit.servo[7].angle = max(PAN_MIN_SERVO, min(PAN_MAX_SERVO, float(self.pan_angle)))
                # Canal 6 = Tilt (inclinaison sol/ciel : 115??=sol/pince, 88??=horizontal, 50??=ciel)
                self.kit.servo[6].angle = max(TILT_MIN_SERVO, min(TILT_MAX_SERVO, float(self.tilt_angle)))  # tilt : butee basse 30
                # Paupi??res
                self.kit.servo[8].angle = max(0.0, min(180.0, float(self.eyelid_left)))
                self.kit.servo[9].angle = max(0.0, min(180.0, float(self.eyelid_right)))
            except Exception as e:
                pass

    def look(self, pan: float = 0.0, tilt: float = 0.0, duration: float = 0.8):
        """
        Oriente la t??te :
        - pan : angle relatif en degr??s. N??gatif = vers la gauche (-40?? max), Positif = vers la droite (+40?? max).
        - tilt : angle relatif en degr??s. Positif = vers le sol (+38?? = sol/pince, +46?? max), N??gatif = vers le ciel (-25?? max).
        """
        # Neutre Pan = 90??. 
        # Sur Milan 2, Canal 7 : angle d??croissant = droite (60??), angle croissant = gauche (120??).
        # Convention : pan > 0 -> droite, pan < 0 -> gauche
        target_pan = max(PAN_MIN_SERVO, min(PAN_MAX_SERVO, 90.0 - pan))
        # Neutre Tilt = 88??.
        # Sur Milan 2, Canal 6 : angle plus grand = vers le sol (115?? = sol/pince).
        # Angle plus petit = vers le ciel (50?? = plafond).
        # tilt positif = vers le sol, tilt n??gatif = vers le ciel.
        target_tilt = max(TILT_MIN_SERVO, min(TILT_MAX_SERVO, 88.0 - tilt))

        # Vitesse limitee : la duree demandee ne peut pas descendre sous MIN_HEAD_MOVE_S
        # (pour un mouvement lent et doux de la tete, meme si l'appelant demande court).
        span = max(abs(target_pan - self.pan_angle), abs(target_tilt - self.tilt_angle))
        duration_min_speed = span / HEAD_MAX_DEG_PER_S  # duree imposee par la vitesse max
        duration = max(duration, min(duration_min_speed, HEAD_MAX_MOVE_S))

        if duration <= 0:
            self.pan_angle = target_pan
            self.tilt_angle = target_tilt
            self.apply_hardware()
            return
        start_pan = self.pan_angle
        start_tilt = self.tilt_angle
        steps = max(1, int(duration * 30))
        delay = duration / steps
        for step in range(1, steps + 1):
            t = step / steps
            ease = (1.0 - math.cos(math.pi * t)) / 2.0
            self.pan_angle = start_pan + (target_pan - start_pan) * ease
            self.tilt_angle = start_tilt + (target_tilt - start_tilt) * ease
            self.apply_hardware()
            time.sleep(delay)

    def look_ahead_ground(self, duration: float = 0.8):
        """Regard plongeant mod??r?? vers le sol en avant (Canal 6 ?? ~76??, distance 1m ?? 3m)."""
        self.look(pan=0.0, tilt=12.0, duration=duration)

    def look_at_ground(self, duration: float = 0.8):
        """Incline franchement le regard vers le sol devant le robot (Canal 6 ?? 60??)."""
        self.look(pan=0.0, tilt=28.0, duration=duration)

    def look_at_claw_ground(self, duration: float = 0.8):
        """Incline profond??ment le regard vers le sol et la pince (Canal 6 ?? 50??)."""
        self.look(pan=0.0, tilt=58.0, duration=duration)  # servo tilt 30 = regard sol max (butee basse validee)

    def look_straight(self, duration: float = 0.8):
        """Regard droit devant ?? l'horizontale (Canal 7 ?? 90??, Canal 6 ?? 88??)."""
        self.look(pan=0.0, tilt=0.0, duration=duration)

    def set_eyelids(self, left_closed_pct: float, right_closed_pct: float):
        # Gauche : 0?? = ouvert, 70?? = ferm??
        # Droite : 180?? = ouvert, 110?? = ferm??
        self.eyelid_left = 0.0 + (left_closed_pct / 100.0) * 70.0
        self.eyelid_right = 180.0 - (right_closed_pct / 100.0) * 70.0
        self.apply_hardware()

    def blink(self):
        """Fait un clignement d'yeux naturel et rapide."""
        # Fermer
        self.set_eyelids(100.0, 100.0)
        time.sleep(0.12)
        # Rouvrir
        self.set_eyelids(0.0, 0.0)

    def wink(self, left: bool = True):
        """Fait un clin d'??il complice ?? Milan !"""
        if left:
            self.set_eyelids(100.0, 0.0)
        else:
            self.set_eyelids(0.0, 100.0)
        time.sleep(0.35)
        self.set_eyelids(0.0, 0.0)

    def sleep_mode(self):
        """Ferme les yeux pour s'endormir."""
        self.set_eyelids(100.0, 100.0)
        self.look(pan=0.0, tilt=15.0)  # Baisse un peu la t??te vers le sol

    def wake_up(self):
        """Se r??veille et ouvre grand les yeux."""
        self.look_straight()
        time.sleep(0.2)
        self.blink()
        time.sleep(0.1)
        self.set_eyelids(0.0, 0.0)













