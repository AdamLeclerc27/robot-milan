"""
Sequence de saisie validee physiquement (fleche au sol, base centree a 64 deg).

Positions de reference :
- PARK (nouvelle) : poignet a 90 deg pour proteger la webcam de la tourelle.
- Tete : tilt 58 deg, pan 0, regard vers le sol pour voir la fleche.
- Fleche dans la vue de tete (park) : centroide ~ (474, 909) sur 0-1000.
- Base de centrage : 64 deg (plage validee 60-65).
- Pose avant saisie : [64, 42, 97, 116, 145, 20] (pince ouverte).
- Saisie : fermeture de la pince a 90 deg, puis levee de l'epaule.
"""
import time

PARK_POSE = [180, 110, 150, 90, 152, 88]
HEAD_TILT_GROUND = 58.0
HEAD_PAN_CENTER = 0.0
BASE_CENTER_DEG = 64.0
PRE_GRAB_POSE = [64.0, 42.0, 97.0, 116.0, 145.0, 20.0]
GRIP_CLOSED_DEG = 90.0
LIFT_SHOULDER_DEG = 22.0
LIFT_DURATION = 1.5


def run_grab_sequence(arm, head, claw_camera=None, log=print):
    """Positionne le bras en park, tourne la base vers le centre, descend vers la pose de saisie
    (l'epaule en dernier), ferme la pince puis leve l'epaule."""
    log('[GRAB] Positionnement tete et bras en park')
    head.look(pan=HEAD_PAN_CENTER, tilt=HEAD_TILT_GROUND, duration=0.8)
    arm.move_joints(PARK_POSE, duration=1.2)
    time.sleep(0.3)

    log(f'[GRAB] Rotation base vers {BASE_CENTER_DEG} deg (centrage fleche)')
    arm.move_joints({0: BASE_CENTER_DEG}, duration=1.0)
    time.sleep(0.3)

    log('[GRAB] Descente vers la pose de saisie (epaule en dernier)')
    arm.move_joints({4: PRE_GRAB_POSE[4], 2: PRE_GRAB_POSE[2], 3: PRE_GRAB_POSE[3], 5: PRE_GRAB_POSE[5]}, duration=1.5)
    arm.move_joints({1: PRE_GRAB_POSE[1]}, duration=1.5)
    time.sleep(0.5)

    log(f'[GRAB] Fermeture de la pince a {GRIP_CLOSED_DEG} deg')
    arm.close_gripper(angle=GRIP_CLOSED_DEG, duration=0.8)
    time.sleep(0.5)

    log(f'[GRAB] Levee de l epaule a {LIFT_SHOULDER_DEG} deg')
    arm.move_joints({1: LIFT_SHOULDER_DEG}, duration=LIFT_DURATION)
    time.sleep(0.5)
    return True
