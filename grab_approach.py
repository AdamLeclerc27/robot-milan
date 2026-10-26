"""
Sous-mission 1 : APPROCHE (sans rotation du chassis).

La fleche doit etre dans la zone de saisie de la camera de tete, telle que validee :
- centroide ~ (cx 494, cy 847) sur 0-1000, bras en park, base de centrage a 64 deg.
La mission ne tourne JAMAIS le chassis pour recentrer : seule la tete bouge.
Si la fleche n'est pas dans la zone, la mission s'arrete et le signale.
"""

TARGET_CX = 494      # position x validee de la fleche dans la camera de tete (0..1000)
TARGET_CY = 847      # position y validee de la fleche dans la camera de tete (0..1000)
TOL_CX = 60          # tolerance horizontale
TOL_CY = 60          # tolerance verticale
MIN_AREA = 80        # surface minimale d'une detection valide (pixels)


def locate_dart_head(frame, detector):
    """Retourne (cx, cy) sur 0-1000, ou None si la fleche n'est pas vue."""
    res = detector(frame)
    if not res or not res.get('found'):
        return None
    return res['cx'], res['cy']


def in_grab_zone(cx, cy):
    return abs(cx - TARGET_CX) <= TOL_CX and abs(cy - TARGET_CY) <= TOL_CY


def run_approach(head, get_frame, detector, log=print):
    """Verifie que la fleche est dans la zone de saisie, sans bouger le chassis.
    Retourne True si la fleche est dans la zone, False sinon."""
    log('[APPROCHE] Verification de la fleche dans la zone de saisie (sans rotation du chassis)')
    pos = locate_dart_head(get_frame(), detector)
    if pos is None:
        log('[APPROCHE] Fleche non vue par la camera de tete : arret')
        return False
    cx, cy = pos
    log(f'[APPROCHE] Fleche vue : cx={cx} cy={cy} (cible cx={TARGET_CX} cy={TARGET_CY})')
    if in_grab_zone(cx, cy):
        log('[APPROCHE] Fleche dans la zone de saisie : pret pour la saisie')
        return True
    log('[APPROCHE] Fleche hors zone de saisie : aucune rotation du chassis, arret')
    return False
