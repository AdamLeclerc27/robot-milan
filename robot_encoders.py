"""
Encodeurs à effet Hall (quadrature A/B) des roues gauche et droite de Milan 2.

Caractéristiques :
  - 11 impulsions par tour moteur (par canal), décodées en quadrature x4 -> 44 transitions/tour moteur
  - Réducteur 46.8:1  -> 44 * 46.8 = 2059.2 transitions par tour de roue
  - Roue Ø 65 mm      -> périmètre 0.2042 m -> ~10 084 transitions par mètre

Câblage (numérotation BCM, broche physique entre parenthèses) :
  - GAUCHE : Phase A = BCM 5 (Phys 29)  | Phase B = BCM 6 (Phys 31)
  - DROIT  : Phase A = BCM 13 (Phys 33) | Phase B = BCM 19 (Phys 35)

Utilise gpiozero (comme robot_drive.py). Les broches sont distinctes de celles des moteurs.
"""
import math
import threading
import time

from gpiozero import DigitalInputDevice

# ===== Configuration Hardware (BCM) =====
LEFT_A = 5
LEFT_B = 6
RIGHT_A = 13
RIGHT_B = 19

# Sens de comptage par roue (+1 ou -1), calibré pour que la marche avant soit positive.
LEFT_SIGN = +1
RIGHT_SIGN = -1

# ===== Caractéristiques mécaniques =====
PULSES_PER_MOTOR_REV = 11
QUADRATURE_FACTOR = 4
GEAR_RATIO = 46.8
WHEEL_DIAMETER_M = 0.065

COUNTS_PER_MOTOR_REV = PULSES_PER_MOTOR_REV * QUADRATURE_FACTOR        # 44
COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO               # 2059.2
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M                     # ~0.2042 m
COUNTS_PER_METER = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_M        # ~10 084

# Pull-up interne : adapté aux capteurs Hall à sortie open-collector (niveau bas = actif).
PULL_UP = True

# Transitions de quadrature (état précédent, état suivant) -> sens de rotation
# état = (A << 1) | B  ->  00, 01, 11, 10 forment un cycle complet
_QUAD_STEP = {
    (0b00, 0b01): +1, (0b01, 0b11): +1, (0b11, 0b10): +1, (0b10, 0b00): +1,
    (0b00, 0b10): -1, (0b10, 0b11): -1, (0b11, 0b01): -1, (0b01, 0b00): -1,
}


def counts_to_meters(counts):
    """Convertit un nombre de transitions en distance parcourue par la roue (m)."""
    return counts / COUNTS_PER_METER


def meters_to_counts(meters):
    """Convertit une distance (m) en nombre de transitions à compter."""
    return meters * COUNTS_PER_METER


def _claim_input(pin, attempts=10, delay=0.5):
    """
    Réserve une broche d'entrée. Réessaie si le handle lgpio précédent n'est pas encore relâché
    (erreur 'GPIO busy' transitoire observée juste après la fermeture d'un autre processus).
    """
    for i in range(attempts):
        try:
            return DigitalInputDevice(pin, pull_up=PULL_UP, bounce_time=None)
        except Exception as e:
            if i == attempts - 1 or "busy" not in str(e).lower():
                raise
            time.sleep(delay)


def _read_level(device):
    """Niveau logique physique (0 ou 1) de la broche, en tenant compte du pull-up."""
    return int(not device.is_active) if PULL_UP else int(device.is_active)


class WheelEncoder:
    """Compte les transitions d'un encodeur à effet Hall en quadrature (A/B) via interruptions.

    sign : +1 ou -1. Permet d'orienter le compte pour que la marche avant soit positive.
    """

    def __init__(self, name, pin_a, pin_b, sign=1):
        self.name = name
        self.sign = sign
        self.count = 0       # Nombre net de transitions (signé, orienté avant = positif)
        self.direction = 0   # Dernier sens observé : +1, -1 ou 0 (aucun mouvement)
        self.invalid = 0     # Transitions impossibles (bruit ou fronts simultanés)
        self._lock = threading.Lock()

        self.dev_a = _claim_input(pin_a)
        self.dev_b = _claim_input(pin_b)
        # Niveaux mémorisés séparément : chaque front ne met à jour que son propre bit.
        self._a = _read_level(self.dev_a)
        self._b = _read_level(self.dev_b)

        self.dev_a.when_activated = self._on_a
        self.dev_a.when_deactivated = self._on_a
        self.dev_b.when_activated = self._on_b
        self.dev_b.when_deactivated = self._on_b

    def _update(self, new_a, new_b):
        """Applique un nouvel état (A, B) mémorisé. Doit être appelé sous verrou."""
        old_state = (self._a << 1) | self._b
        new_state = (new_a << 1) | new_b
        self._a, self._b = new_a, new_b
        if new_state == old_state:
            return
        step = _QUAD_STEP.get((old_state, new_state))
        if step is None:
            self.invalid += 1
        else:
            self.count += step * self.sign
            self.direction = step * self.sign

    def _on_a(self):
        with self._lock:
            self._update(_read_level(self.dev_a), self._b)

    def _on_b(self):
        with self._lock:
            self._update(self._a, _read_level(self.dev_b))

    def snapshot(self):
        """Retourne (compte, sens, invalides, niveau A, niveau B) de façon cohérente."""
        with self._lock:
            return self.count, self.direction, self.invalid, self._a, self._b

    def reset(self):
        """Remet le compte à zéro (le compteur d'invalides est conservé)."""
        with self._lock:
            self.count = 0

    def close(self):
        self.dev_a.close()
        self.dev_b.close()


class WheelEncoders:
    """Paire d'encodeurs : roue gauche et roue droite."""

    def __init__(self):
        # Mesures à l'arrêt/marche : le compte gauche est positif en marche avant, le droit négatif.
        # Le signe du droit est inversé pour que la marche avant donne + sur les deux roues.
        self.left = WheelEncoder("GAUCHE", LEFT_A, LEFT_B, sign=LEFT_SIGN)
        self.right = WheelEncoder("DROIT", RIGHT_A, RIGHT_B, sign=RIGHT_SIGN)

    def reset(self):
        self.left.reset()
        self.right.reset()

    def counts(self):
        """Retourne (compte gauche, compte droit)."""
        return self.left.snapshot()[0], self.right.snapshot()[0]

    def wheel_distances_m(self):
        """Distance signee parcourue par chaque roue depuis le dernier reset : (gauche, droite) en m."""
        cl, cr = self.counts()
        return counts_to_meters(cl), counts_to_meters(cr)

    def distance_m(self):
        """Distance moyenne parcourue depuis le dernier reset (signée, en m)."""
        cl, cr = self.counts()
        return counts_to_meters((cl + cr) / 2.0)

    def close(self):
        self.left.close()
        self.right.close()
