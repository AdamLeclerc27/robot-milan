"""
Validation et correction automatique de la prise.

Apres la descente (grab_sequence), la mini camera de pince doit voir la fleche
dans la zone de prise. Si elle est hors zone, on corrige la base de petits pas
(pas de roues), on redescend la pose de prise, et on recommence (max N essais).
"""
import time

CLAW_CENTER = (336, 298)  # zone de prise (pixels, camera 640x480), voir arm_config.CLAW_CAM_GRAB_VIEW
CLAW_W, CLAW_H = 640, 480
TOL_X = 150               # tolerance horizontale (0..1000)
TOL_Y = 200               # tolerance verticale (0..1000)
MIN_BLUE_PX = 150         # pixels bleus minimum pour considerer la fleche visible
BASE_STEP_DEG = 1.0       # pas de correction de la base (degres)
BASE_MIN, BASE_MAX = 40.0, 100.0
MAX_ATTEMPTS = 3

center_n_x = round(CLAW_CENTER[0] / CLAW_W * 1000)
center_n_y = round(CLAW_CENTER[1] / CLAW_H * 1000)


def blue_position(frame):
    """Retourne (n_pixels, cx, cy) sur 0-1000 pour le bleu de la fleche, ou (0, 0, 0)."""
    import cv2
    import numpy as np
    if frame is None:
        return 0, 0, 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([90, 60, 40]), np.array([135, 255, 255]))
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return 0, 0, 0
    h, w = frame.shape[:2]
    return len(xs), int(xs.mean() / w * 1000), int(ys.mean() / h * 1000)


def in_claw_zone(cx, cy):
    return abs(cx - center_n_x) <= TOL_X and abs(cy - center_n_y) <= TOL_Y


def validate_and_correct(arm, claw_frame_fn, log=print):
    """Verifie la fleche dans la pince et corrige la base jusqu'a MAX_ATTEMPTS.
    Retourne True si la fleche est dans la zone, False sinon (arret sans mouvement hasardeux)."""
    base = arm.current_angles[0]
    for attempt in range(1, MAX_ATTEMPTS + 1):
        n, cx, cy = blue_position(claw_frame_fn())
        log(f'[CORRECT] essai {attempt}: fleche px={n} cx={cx} cy={cy} (zone cx={center_n_x} cy={center_n_y})')
        if n < MIN_BLUE_PX:
            log('[CORRECT] fleche non visible dans la pince : arret, pas de correction a l aveugle')
            return False
        if in_claw_zone(cx, cy):
            log('[CORRECT] fleche dans la zone de prise : validation OK')
            return True
        # Ecart horizontal -> direction de la base (cx croissant = vers la droite de la camera)
        err = cx - center_n_x
        step = BASE_STEP_DEG if err < 0 else -BASE_STEP_DEG
        new_base = min(BASE_MAX, max(BASE_MIN, base + step))
        if new_base == base:
            log('[CORRECT] limite de base atteinte : arret')
            return False
        log(f'[CORRECT] correction base {base:.1f} -> {new_base:.1f}')
        arm.move_joints({0: new_base}, duration=0.6)
        time.sleep(0.3)
        base = new_base
    log('[CORRECT] nombre maximal d essais atteint : arret')
    return False
