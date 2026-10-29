"""
Cinematique du bras Milan 2 : SOURCE UNIQUE DE VERITE.

Convention des servos (voir la procedure de montage) :
  Chaque servo est monte a 90 deg dans sa position de reference geometrique.
  - servo 0 (base)    : 90 = bras dans le plan AVANT (phi = servo - 90 ; + = gauche)
  - servo 1 (epaule)  : 90 = bras-haut VERTICAL       (theta1 = servo - 90 ; + = penche vers l'avant)
  - servo 2 (coude)   : 90 = avant-bras dans l'axe      (theta2 = theta1 + (servo - 90))
  - servo 3 (poignet) : 90 = pince dans l'axe           (theta3 = theta2 + (servo - 90))
  Les angles theta sont CUMULES : ce sont des inclinaisons absolues, mesurees depuis la verticale.

Repere : origine = axe de rotation de la tourelle, sur la surface du support.
  X = avant, Y = gauche, Z = hauteur au-dessus de la surface.

Point de travail = POINTE de la pince (TCP). Les longueurs sont mesurees d'axe a axe.
"""

import math
from typing import Dict, Tuple

# --- Geometrie (cm) : d'axe a axe ---
L_SHOULDER = 10.0      # epaule -> coude
L_ELBOW = 10.0         # coude -> poignet
L_WRIST_TIP = 9.68     # poignet -> pointe de la pince
SHOULDER_HEIGHT = 15.0   # hauteur de l'axe de la tourelle / epaule au-dessus de la surface

# Servo de reference (position mecanique a 90 deg)
SERVO_REF = 90.0

# Limites des servos (deg). A ajuster si la mecanique l'exige.
SERVO_MIN = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0}
SERVO_MAX = {0: 180.0, 1: 180.0, 2: 180.0, 3: 180.0, 4: 180.0, 5: 180.0}


def inclinations(servo_epaule: float, servo_coude: float, servo_poignet: float) -> Tuple[float, float, float]:
    """Angles geometriques (deg, depuis la verticale) des trois segments du plan du bras."""
    theta1 = servo_epaule - SERVO_REF
    theta2 = theta1 + (servo_coude - SERVO_REF)
    theta3 = theta2 + (servo_poignet - SERVO_REF)
    return theta1, theta2, theta3


def forward(servo_base: float, servo_epaule: float, servo_coude: float, servo_poignet: float) -> Dict[str, Tuple[float, float, float]]:
    """
    Cinematique directe. Retourne les positions (x, y, z) en cm, repere tourelle, de chaque articulation
    et de la pointe. Z = hauteur au-dessus de la surface.
    """
    phi = math.radians(servo_base - SERVO_REF)          # rotation de la tourelle
    t1, t2, t3 = (math.radians(a) for a in inclinations(servo_epaule, servo_coude, servo_poignet))

    # Dans le plan du bras : u = distance horizontale (vers l'avant), v = hauteur au-dessus de l'epaule
    u_e, v_e = L_SHOULDER * math.sin(t1), L_SHOULDER * math.cos(t1)
    u_w = u_e + L_ELBOW * math.sin(t2)
    v_w = v_e + L_ELBOW * math.cos(t2)
    u_t = u_w + L_WRIST_TIP * math.sin(t3)
    v_t = v_w + L_WRIST_TIP * math.cos(t3)

    def world(u: float, v: float) -> Tuple[float, float, float]:
        # Le plan du bras tourne autour de Z de phi. Phi = 0 : avant (+X). Phi > 0 : vers la gauche (+Y).
        return (u * math.cos(phi), u * math.sin(phi), SHOULDER_HEIGHT + v)

    return {
        'epaule': world(0.0, 0.0),
        'coude': world(u_e, v_e),
        'poignet': world(u_w, v_w),
        'pointe': world(u_t, v_t),
    }


def tip(servo_base: float, servo_epaule: float, servo_coude: float, servo_poignet: float) -> Tuple[float, float, float]:
    """Position de la pointe (cm) : c'est le point de travail."""
    return forward(servo_base, servo_epaule, servo_coude, servo_poignet)['pointe']


def inverse_all(x: float, y: float, z: float, pince_deg: float = 180.0) -> list:
    """
    Toutes les solutions analytiques (coude en haut ET coude en bas), sans filtrage de bornes servo.
    Chaque solution est un dict {'base','epaule','coude','poignet'}. Liste vide si hors portee geometrique.
    """
    sols = []
    phi = math.atan2(y, x)
    r_tip = math.hypot(x, y)
    base = SERVO_REF + math.degrees(phi)
    t3 = math.radians(pince_deg)
    u_w = r_tip - L_WRIST_TIP * math.sin(t3)
    v_w = (z - SHOULDER_HEIGHT) - L_WRIST_TIP * math.cos(t3)
    d2 = u_w * u_w + v_w * v_w
    d = math.sqrt(d2)
    if d > L_SHOULDER + L_ELBOW or d < abs(L_SHOULDER - L_ELBOW):
        return sols
    c_coude = max(-1.0, min(1.0, (d2 - L_SHOULDER ** 2 - L_ELBOW ** 2) / (2.0 * L_SHOULDER * L_ELBOW)))
    gamma_abs = math.acos(c_coude)
    alpha = math.atan2(u_w, v_w)
    for gamma in (gamma_abs, -gamma_abs):   # coude en haut puis en bas
        beta = math.atan2(L_ELBOW * math.sin(gamma), L_SHOULDER + L_ELBOW * math.cos(gamma))
        t1 = alpha - beta
        u_e, v_e = L_SHOULDER * math.sin(t1), L_SHOULDER * math.cos(t1)
        t2 = math.atan2(u_w - u_e, v_w - v_e)
        sols.append({
            'base': base,
            'epaule': SERVO_REF + math.degrees(t1),
            'coude': SERVO_REF + math.degrees(t2 - t1),
            'poignet': SERVO_REF + math.degrees(t3 - t2),
        })
    return sols


def inverse(x: float, y: float, z: float, pince_deg: float = 180.0) -> Dict[str, float]:
    """
    Cinematique inverse analytique.
      x, y, z : pointe cible (cm), repere tourelle (z = hauteur au-dessus de la surface).
      pince_deg : inclinaison ABSOLUE de la pince depuis la verticale, en deg.
                  180 = pince verticale pointe vers le bas. 90 = horizontale.
    Retourne les 4 servos (base, epaule, coude, poignet) ou leve ValueError si la cible est hors portee.
    Coude en haut (solution unique retenue).
    """
    # 1) Base : direction horizontale de la cible
    phi = math.atan2(y, x)
    r_tip = math.hypot(x, y)
    base = SERVO_REF + math.degrees(phi)

    # 2) Ramener la cible au POIGNET : la pince fait un angle t3 avec la verticale.
    #    Dans le plan du bras (u = distance horizontale, v = hauteur sur l'epaule)
    t3 = math.radians(pince_deg)
    u_t = r_tip
    v_t = z - SHOULDER_HEIGHT
    u_w = u_t - L_WRIST_TIP * math.sin(t3)
    v_w = v_t - L_WRIST_TIP * math.cos(t3)

    # 3) Epaule-coude-poignet : loi des cosinus
    d2 = u_w * u_w + v_w * v_w
    d = math.sqrt(d2)
    if d > L_SHOULDER + L_ELBOW or d < abs(L_SHOULDER - L_ELBOW):
        raise ValueError("cible hors portee (poignet a %.2f cm de l'epaule)" % d)

    c_coude = (d2 - L_SHOULDER ** 2 - L_ELBOW ** 2) / (2.0 * L_SHOULDER * L_ELBOW)
    c_coude = max(-1.0, min(1.0, c_coude))
    # Angle interne du coude (0 = bras replie, pi = bras tendu) : coude en haut
    gamma = math.acos(c_coude)

    # Angle de l'epaule (inclinaison du bras-haut depuis la verticale), coude en haut
    alpha = math.atan2(u_w, v_w)                                   # direction epaule -> poignet
    beta = math.atan2(L_ELBOW * math.sin(gamma), L_SHOULDER + L_ELBOW * math.cos(gamma))
    t1 = alpha - beta

    # Inclinaison de l'avant-bras (absolue, depuis la verticale) : direction coude -> poignet
    u_e, v_e = L_SHOULDER * math.sin(t1), L_SHOULDER * math.cos(t1)
    t2 = math.atan2(u_w - u_e, v_w - v_e)

    # Servos (convention cumulee : theta2 = theta1 + (coude - 90), theta3 = theta2 + (poignet - 90))
    shoulder = SERVO_REF + math.degrees(t1)
    elbow = SERVO_REF + math.degrees(t2 - t1)
    wrist = SERVO_REF + math.degrees(t3 - t2)

    return {'base': base, 'epaule': shoulder, 'coude': elbow, 'poignet': wrist}
