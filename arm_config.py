"""
Configuration validée et définitive du bras robotique 6-DOF (ROT3U) pour Milan 2.

Géométrie physique retenue :
- Fixation : Torse avant du robot (sur le seau de 5 gallons)
- Hauteur de fixation : 15 cm du sol (sur support horizontal)
- Segment 0 (Base)     : Tourelle rotative (Z Yaw), axe vertical
- Segment 1 (Épaule)   : 10 cm (Pitch Y)
- Segment 2 (Coude)    : 10 cm (Pitch Y)
- Segment 3 (Poignet)  : 8 cm (Pitch Y + Roll + Pince)
"""

# Plage d'impulsion sécurisée pour MG996R (évite la surchauffe et les butées mécaniques)
PULSE_MIN = 750
PULSE_MAX = 2250

# Limites angulaires physiques réelles par canal (0 à 5)
ARM_LIMITS = {
    0: {"name": "Base (Tourelle)",    "min": 50, "max": 160, "home": 110, "offset": 20},
    1: {"name": "Épaule (Pitch)",     "min": 0,  "max": 180, "home": 45,  "offset": 0},
    2: {"name": "Coude (Pitch)",      "min": 10, "max": 150, "home": 90,  "offset": 0},
    3: {"name": "Poignet (Pitch)",    "min": 20, "max": 180, "home": 90,  "offset": 0},
    4: {"name": "Rotation Poignet",   "min": 0,  "max": 180, "home": 90,  "offset": 0},
    5: {"name": "Pince",              "min": 20, "max": 85,  "home": 20,  "offset": 0}  # 20=ouverte, 85=fermée
}

# Dimensions physiques en mètres
LINK_LENGTHS = {
    "mount_height": 0.15,   # 15 cm : Hauteur de fixation du support au-dessus du sol
    "base_height":  0.08,   # 8 cm  : Axe de la tourelle à l'axe de l'épaule
    "shoulder":     0.10,   # 10 cm : Axe épaule -> Axe coude
    "elbow":        0.10,   # 10 cm : Axe coude -> Axe poignet
    "wrist_tip":    0.08    # 8 cm  : Axe poignet -> Extrémité de la pince
}

# Poses clés verrouillées et validées (en degrés pour les canaux 0 à 5)
# Format : [base, epaule, coude, poignet, rotation, pince]
ARM_POSES = {
    # 1. Plié compact serré contre le seau pour rouler sans basculer
    "park": [110, 15, 140, 130, 90, 85],
    "park_compact": [110, 15, 140, 130, 90, 85],

    # 2. Ramassage au sol validé (Z ~ 2.2 cm du sol, X ~ 33.6 cm du centre)
    "ground_grab": [110, 135, 75, 129, 86, 20],

    # 3. Présentation haute à hauteur des yeux (~50 cm de haut, devant webcam et Jabra)
    "eye_level": [110, 25, 70, 90, 90, 80],
    "observe": [110, 25, 70, 90, 90, 20],

    # 4. Tendu droit devant à l'horizontale (mi-hauteur)
    "forward_reach": [110, 85, 90, 90, 90, 30],

    # 5. Position neutre / attente
    "home": [110, 45, 90, 90, 90, 20],

    # 6. Transport d'un objet saisi
    "carry": [110, 40, 110, 110, 90, 80]
}
