"""
Module de pilotage différentiel des roues avec correction de dérive en ligne droite.
Milan 2 : Châssis aspirateur robot (Moteurs gauche GPIO 23,22 et droit GPIO 18,17).

Fonctionnalités :
- Correction d'asymétrie mécanique (Trim gauche/droit pour éliminer l'effet 'banane').
- Maintien de cap en ligne droite (Heading Hold).
- Accélération progressive (Ramp / Anti-patinage) pour ne pas secouer le bras et le seau.
- Mode simulation automatique sur PC.
"""

import time
import math
from typing import Optional

# Entraxe des roues (m) : distance entre les centres des roues gauche et droite.
# Mesure sur le robot : ~23-24 cm (roues centrees sur le chassis). Utilisee par turn_by_encoders().
TRACK_WIDTH_M = 0.235

def hold_motors_low(pins=(17, 18, 22, 23)):
    """
    Force les broches moteur à LOW via pinctrl (persistant après la fin du processus,
    contrairement à gpiozero qui relâche la broche : GPIO23 remonte alors à HAUT et la roue tourne).
    Retourne True si la commande a réussi.
    """
    try:
        import subprocess
        spec = ",".join(str(p) for p in pins)
        subprocess.run(["pinctrl", "set", spec, "op", "dl"], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
        return True
    except Exception as e:
        print(f"[RobotDrive] ⚠️ Impossible de forcer les moteurs à LOW ({e}).")
        return False


try:
    from gpiozero import Robot as GpioRobot
    GPIO_AVAILABLE = True
except (ImportError, Exception):
    GPIO_AVAILABLE = False


class MockGpioRobot:
    """Simulateur de robot roulant pour tests sur PC."""
    def __init__(self, left=(23, 22), right=(18, 17)):
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.is_moving = False

    def forward(self, speed=1.0, curve_left=0.0, curve_right=0.0):
        self.left_speed = max(0.0, speed - curve_left)
        self.right_speed = max(0.0, speed - curve_right)
        self.is_moving = True

    def backward(self, speed=1.0, curve_left=0.0, curve_right=0.0):
        self.left_speed = -speed
        self.right_speed = -speed
        self.is_moving = True

    def left(self, speed=0.75):
        self.left_speed = -speed
        self.right_speed = speed
        self.is_moving = True

    def right(self, speed=0.75):
        self.left_speed = speed
        self.right_speed = -speed
        self.is_moving = True

    def stop(self):
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.is_moving = False


class RobotDrive:
    def __init__(self, left_pins=(23, 22), right_pins=(18, 17), trim: float = 0.0, encoders=None):
        """
        trim: Coefficient d'équilibrage entre roues (-1.0 à +1.0).
              Si trim > 0 : la roue gauche pousse plus fort que la droite (on compense en ralentissant la gauche).
              Si trim < 0 : la roue droite pousse plus fort que la gauche (on compense en ralentissant la droite).
        """
        self.trim = trim
        self.speed_mps = 0.40  # Vitesse nominale mesurée (~40 cm/seconde)
        self.turn_rate_dps = 120.0  # Vitesse de rotation (~120 degrés/seconde)
        # encoders : instance de robot_encoders.WheelEncoders (optionnel).
        # Si fourni, drive_distance() mesure la distance réelle au lieu de la estimer par le temps.
        self.encoders = encoders
        self._abort = False  # Positionné par stop() pour interrompre une avance en cours

        if GPIO_AVAILABLE:
            try:
                self.robot = GpioRobot(left=left_pins, right=right_pins)
                print(f"[RobotDrive] ✅ Moteurs GPIO initialisés (G:{left_pins}, D:{right_pins}).")
            except Exception as e:
                print(f"[RobotDrive] ⚠️ Erreur GPIO ({e}). Bascule en simulation.")
                self.robot = MockGpioRobot()
        else:
            print("[RobotDrive] ℹ️ Mode Simulation actif pour les roues.")
            self.robot = MockGpioRobot()

    def set_trim(self, trim_val: float):
        """Ajuste la compensation de trajectoire (-0.3 à +0.3)."""
        self.trim = max(-0.3, min(0.3, float(trim_val)))
        print(f"[RobotDrive] 🎯 Compensation de dérive (Trim) ajustée à : {self.trim:+.3f}")

    def _get_balanced_speeds(self, base_speed: float) -> tuple:
        """Calcule les vitesses équilibrées gauche/droite pour une ligne droite parfaite."""
        if self.trim > 0:
            # Roue gauche a tendance à aller trop vite -> on la freine
            v_left = base_speed * (1.0 - self.trim)
            v_right = base_speed
        elif self.trim < 0:
            # Roue droite va trop vite -> on la freine
            v_left = base_speed
            v_right = base_speed * (1.0 + self.trim)
        else:
            v_left = base_speed
            v_right = base_speed

        return max(0.0, min(1.0, v_left)), max(0.0, min(1.0, v_right))

    def forward(self, speed: float = 0.85):
        """Avance en ligne droite avec compensation de dérive."""
        v_l, v_r = self._get_balanced_speeds(speed)
        # gpiozero Robot accepte curve_left / curve_right
        if v_l < v_r:
            curve = (v_r - v_l) / v_r
            self.robot.forward(v_r, curve_left=curve)
        elif v_r < v_l:
            curve = (v_l - v_r) / v_l
            self.robot.forward(v_l, curve_right=curve)
        else:
            self.robot.forward(speed)

    def curve_forward(self, speed: float, steering: float):
        """
        Avance en tournant doucement.
        steering < 0 : tourne à gauche.
        steering > 0 : tourne à droite.
        -1.0 <= steering <= 1.0
        """
        v_l, v_r = self._get_balanced_speeds(speed)
        
        # Appliquer la direction
        if steering < 0:
            # Tourne à gauche : on ralentit la roue gauche
            v_l = v_l * (1.0 - abs(steering))
        elif steering > 0:
            # Tourne à droite : on ralentit la roue droite
            v_r = v_r * (1.0 - abs(steering))
            
        if v_l < v_r:
            curve = (v_r - v_l) / v_r
            self.robot.forward(v_r, curve_left=curve)
        elif v_r < v_l:
            curve = (v_l - v_r) / v_l
            self.robot.forward(v_l, curve_right=curve)
        else:
            self.robot.forward(max(v_l, v_r))

    def backward(self, speed: float = 0.75):
        """Recule en ligne droite équilibrée."""
        v_l, v_r = self._get_balanced_speeds(speed)
        if v_l < v_r:
            curve = (v_r - v_l) / v_r
            self.robot.backward(v_r, curve_left=curve)
        elif v_r < v_l:
            curve = (v_l - v_r) / v_l
            self.robot.backward(v_l, curve_right=curve)
        else:
            self.robot.backward(speed)

    def left(self, speed: float = 0.80):
        """Pivote sur place vers la gauche avec couple moteur vigoureux."""
        self.robot.left(speed)

    def right(self, speed: float = 0.80):
        """Pivote sur place vers la droite avec couple moteur vigoureux."""
        self.robot.right(speed)

    def stop(self):
        """Arrêt immédiat des moteurs (interrompt aussi une avance en cours)."""
        self._abort = True
        self.robot.stop()

    def close(self):
        """
        Arrêt propre pour un script de test : coupe les moteurs, libère gpiozero,
        puis force les broches moteur à LOW (sinon GPIO23 remonte et la roue repart).
        """
        self.stop()
        try:
            self.robot.close()
        except Exception:
            pass
        hold_motors_low()

    def drive_distance(self, meters: float, speed: float = 0.75):
        """
        Avance d'une distance précise en mètres.
        - Avec encodeurs : boucle fermée sur la distance mesurée (moyenne des deux roues).
        - Sans encodeurs : repli sur une durée estimée (speed_mps).
        Un stop() pendant l'avance interrompt le mouvement.
        """
        if meters <= 0:
            return
        self._abort = False

        if self.encoders is None:
            duration = meters / self.speed_mps
            print(f"[RobotDrive] 🚗 Avance de {meters*100:.0f} cm en ligne droite ({duration:.1f}s, sans encodeurs)...")
            self.forward(speed)
            time.sleep(duration)
            self.stop()
            print("[RobotDrive] ⏹️ Arrivé à destination.")
            return

        # Délai de sécurité : 2 s + temps à 10 cm/s minimum (vitesse très inférieure à la réelle)
        timeout = 2.0 + meters / 0.10
        print(f"[RobotDrive] 🚗 Avance de {meters*100:.0f} cm (encodeurs, timeout {timeout:.1f}s)...")
        self.encoders.reset()
        self.forward(speed)
        t0 = time.time()
        try:
            while not self._abort:
                if self.encoders.distance_m() >= meters:
                    break
                if time.time() - t0 > timeout:
                    print("[RobotDrive] ⚠️ Timeout : distance cible non atteinte.")
                    break
                time.sleep(0.01)
        finally:
            self.stop()
        print(f"[RobotDrive] ⏹️ Arrêt : {self.encoders.distance_m():.3f} m mesurés (cible {meters:.3f} m).")

    def encoder_status(self):
        """Retourne l'état des encodeurs (None si non branchés)."""
        if self.encoders is None:
            return None
        cl, cr = self.encoders.counts()
        return {
            "gauche": cl,
            "droit": cr,
            "distance_m": round(self.encoders.distance_m(), 4),
            "invalides_gauche": self.encoders.left.invalid,
            "invalides_droit": self.encoders.right.invalid,
            # Diagnostic : niveaux bruts des phases A/B (0/1) et sens du dernier front, pour voir si le signal bouge
            "niveaux_gauche": list(self.encoders.left.snapshot()[3:5]),
            "niveaux_droit": list(self.encoders.right.snapshot()[3:5]),
        }

    def turn_degrees(self, degrees: float, speed: float = 0.65):
        """Pivote sur place d'un angle précis en degrés (+ = droite, - = gauche)."""
        duration = abs(degrees) / self.turn_rate_dps
        if degrees > 0:
            print(f"[RobotDrive] 🔄 Pivote à droite de {degrees:.0f}°...")
            self.right(speed)
        else:
            print(f"[RobotDrive] 🔄 Pivote à gauche de {abs(degrees):.0f}°...")
            self.left(speed)
        time.sleep(duration)
        self.stop()
        print("[RobotDrive] ⏹️ Orientation terminée.")

    def turn_by_encoders(self, degrees: float, speed: float = 0.70, timeout: Optional[float] = None) -> Optional[float]:
        """
        Pivote sur place d'un angle mesure aux encodeurs (+ = droite, - = gauche), en boucle fermee.
        Angle = (distance roue gauche - distance roue droite) / entraxe (TRACK_WIDTH_M).
        Retourne l'angle reellement mesure (deg), ou None si les encodeurs sont absents.
        """
        if self.encoders is None:
            print("[RobotDrive] Pas d'encodeurs : rotation estimee par le temps.")
            self.turn_degrees(degrees, speed)
            return None
        if abs(degrees) < 0.5:
            return 0.0
        self._abort = False
        sign = 1.0 if degrees > 0 else -1.0
        target = abs(degrees)
        if timeout is None:
            timeout = 2.0 + target / 15.0
        self.encoders.reset()
        if sign > 0:
            print(f"[RobotDrive] Pivote a droite de {target:.1f} deg (encodeurs)...")
            self.right(speed)
        else:
            print(f"[RobotDrive] Pivote a gauche de {target:.1f} deg (encodeurs)...")
            self.left(speed)
        t0 = time.time()
        try:
            while not self._abort:
                if sign * self._measured_turn_deg() >= target:
                    break
                if time.time() - t0 > timeout:
                    print("[RobotDrive] Timeout : rotation incomplete.")
                    break
                time.sleep(0.01)
        finally:
            self.stop()
        measured = self._measured_turn_deg()
        # Aucune impulsion d'encodeur : la rotation n'est PAS mesuree. On ne le fait pas croire.
        cl, cr = self.encoders.counts()
        if cl == 0 and cr == 0:
            print(f"[RobotDrive] ECHEC MESURE : aucune impulsion encodeur (roues commandees a {degrees:+.1f} deg). Rotation inconnue.")
            return None
        print(f"[RobotDrive] Rotation mesuree : {measured:+.1f} deg (cible {degrees:+.1f} deg).")
        return measured

    def _measured_turn_deg(self) -> float:
        """Angle de rotation depuis le dernier reset des encodeurs (+ = droite)."""
        dist_l, dist_r = self.encoders.wheel_distances_m()
        return math.degrees((dist_l - dist_r) / TRACK_WIDTH_M)

