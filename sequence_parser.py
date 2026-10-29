"""
Module d'analyse syntaxique et de séquençage pour les commandes vocales de Milan 2.
- Normalise le texte (suppression des accents, ponctuation, casse)
- Détecte les commandes réflexes instantanées (avance, recule, tourne à droite/gauche, pince, etc.)
- Gère le découpage et l'enchaînement des séquences (ex: "avance puis tourne à droite", "avance et regarde devant")
- Détecte l'intention de mission autonome complexe (ex: "ramasse la fléchette sur le plancher et va la déposer dans la boîte")
"""

import re
import unicodedata

def normalize_text(text: str) -> str:
    """Nettoie et normalise le texte pour une comparaison lexicale robuste."""
    if not text:
        return ""
    text = text.lower().strip()
    # Supprimer les accents (é -> e, à -> a, etc.)
    nfkd = unicodedata.normalize('NFKD', text)
    text = ''.join([c for c in nfkd if not unicodedata.combining(c)])
    # Remplacer les caractères non alphanumériques par des espaces
    text = re.sub(r'[^\w\s]', ' ', text)
    # Réduire les espaces multiples
    text = re.sub(r'\s+', ' ', text).strip()
    return text

FRENCH_NUMBERS = {
    "un": 1, "une": 1, "deux": 2, "trois": 3, "quatre": 4, "cinq": 5,
    "six": 6, "sept": 7, "huit": 8, "neuf": 9, "dix": 10
}

def extract_duration(text: str, default: float = 1.0) -> float:
    """Extrait une durée en secondes si spécifiée."""
    if "un peu" in text or "legerement" in text or "petite" in text:
        return max(0.5, default * 0.5)
    if "beaucoup" in text or "longtemps" in text:
        return default * 2.0
    
    # Ex: '2 secondes', '3 s'
    m = re.search(r'(\d+(?:[\.,]\d+)?)\s*(?:seconde|secondes|sec|s\b)', text)
    if m:
        try:
            val = float(m.group(1).replace(',', '.'))
            return min(10.0, max(0.2, val))
        except:
            pass
        
    for word, num in FRENCH_NUMBERS.items():
        if f"{word} seconde" in text or f"{word} sec" in text:
            return float(num)
            
    return default

def match_single_reflex(clause: str):
    """
    Tente de faire correspondre une clause unique à une action réflexe matérielle.
    Retourne (description_vocale, action_dict) ou None.
    """
    c = normalize_text(clause)
    if not c:
        return None
        
    # --- ARRET D'URGENCE ---
    if re.search(r'^(stop|arrete|arreter|arrete toi|pause|stopper|bloque|freine)$', c):
        return ("Je m'arrête.", {"action": "stop", "duration": 0.0})

    # --- CLIGNEMENT / EXPRESSIONS DES YEUX ---
    if re.search(r'^(cligne|cligner|clin d oeil|clin d\'oeil|cligne des yeux)$', c):
        return ("Clin d'œil !", {"action": "blink", "duration": 0.5})

    # --- ORIENTATION DE LA TETE ---
    if re.search(r'^(regarde devant|regarde droit|tete droite|regarde tout droit)$', c):
        return ("Je regarde droit devant.", {"action": "look_forward", "duration": 0.8})
    if re.search(r'^(regarde en bas|regarde par terre|regarde le sol|baisse la tete)$', c):
        return ("Je regarde le sol.", {"action": "look_ground", "duration": 0.8})

    # --- AVANCE ---
    if re.search(r'^(avance|avancer|avancez|en avant|va tout droit|tout droit|avance tout droit|roule)(.*)$', c):
        dur = extract_duration(c, default=1.5)
        if "un peu" in c:
            phrase = "J'avance un peu !"
        elif dur != 1.5:
            phrase = f"J'avance pendant {int(dur)} secondes."
        else:
            phrase = "J'avance !"
        return (phrase, {"action": "forward", "duration": dur})

    # --- RECULE ---
    if re.search(r'^(recule|reculer|reculez|en arriere|marche arriere|fais marche arriere)(.*)$', c):
        dur = extract_duration(c, default=1.5)
        if "un peu" in c:
            phrase = "Je recule un peu !"
        elif dur != 1.5:
            phrase = f"Je recule pendant {int(dur)} secondes."
        else:
            phrase = "Je recule !"
        return (phrase, {"action": "backward", "duration": dur})

    # --- TOURNER A DROITE ---
    if re.search(r'^(?:tourne|tourner|tournez|pivote|pivoter|vire|virer|va)?\s*(?:un\s+peu\s+)?(?:vers|sur|a|de)?\s*(?:la\s*)?droite\b', c) or re.search(r'^(droite|a droite|vers la droite|sur la droite)$', c):
        dur = extract_duration(c, default=0.7)
        if "un peu" in c:
            phrase = "Je tourne un peu à droite."
            dur = 0.35
        elif dur != 0.7:
            phrase = f"Je tourne à droite pendant {int(dur)} secondes."
        else:
            phrase = "Je tourne à droite."
        return (phrase, {"action": "right", "duration": dur})

    # --- TOURNER A GAUCHE ---
    if re.search(r'^(?:tourne|tourner|tournez|pivote|pivoter|vire|virer|va)?\s*(?:un\s+peu\s+)?(?:vers|sur|a|de)?\s*(?:la\s*)?gauche\b', c) or re.search(r'^(gauche|a gauche|vers la gauche|sur la gauche)$', c):
        dur = extract_duration(c, default=0.7)
        if "un peu" in c:
            phrase = "Je tourne un peu à gauche."
            dur = 0.35
        elif dur != 0.7:
            phrase = f"Je tourne à gauche pendant {int(dur)} secondes."
        else:
            phrase = "Je tourne à gauche."
        return (phrase, {"action": "left", "duration": dur})

    # --- DEMI-TOUR ---
    if re.search(r'^(demi tour|fais demi tour|fais un demi tour|retourne toi)$', c):
        return ("Demi-tour !", {"action": "right", "duration": 1.6})

    # --- BRAS / PINCE ---
    if re.search(r'^(ouvre la pince|ouvrir la pince|ouvre la main|lache|relache|ouvre)$', c):
        return ("J'ouvre la pince.", {"action": "open_gripper", "duration": 0.5})
    if re.search(r'^(ferme la pince|fermer la pince|serre la pince|attrape|ferme)$', c):
        return ("Je ferme la pince.", {"action": "close_gripper", "duration": 0.5})

    # --- POSES ARTICULEES DU BRAS ---
    if re.search(r'^(range le bras|replie le bras|bras au repos|position repos|repos)$', c):
        return ("Je range mon bras.", {"action": "park", "duration": 1.2})
    if re.search(r'^(dors|va dormir|bonne nuit|fais dodo|sommeil)$', c):
        return ("Bonne nuit ! Je passe en veille.", {"action": "park", "duration": 1.2})
    if re.search(r'^(reveille toi|debout|reveil|bonjour)$', c):
        return ("Bonjour ! Je suis réveillé !", {"action": "wake_up", "duration": 1.0})
    if re.search(r'^(leve le bras|bras en l air|bras haut|porte)$', c):
        return ("Je range mon bras.", {"action": "park", "duration": 1.2})
    if re.search(r'^(baisse le bras|bras au sol|bras vers le bas)$', c):
        return ("Je range mon bras.", {"action": "park", "duration": 1.2})

    return None

GRAB_VERBS = r"(?:ramasse|ramasser|attrape|attraper|recupere|recuperer|prends|prendre|prend|apporte|apporter|saisis|saisir)"
GIVE_VERBS = r"(?:donne|donner|va donner|apporte a|amene a|porte a)"
DEPOSIT_VERBS = r"(?:depose|deposer|mets|mettre|range|ranger|va deposer|va mettre)"
# Destinations de dépôt génériques (contenants)
CONTAINER_WORDS = ["boite", "bac", "panier", "seau", "carton", "caisse", "corbeille", "table", "sol"]
# Mots qui ne sont jamais une cible d'objet
NOT_OBJECT_WORDS = {"la", "le", "les", "un", "une", "des", "du", "de", "moi", "toi", "lui", "elle", "a", "au", "aux", "dans", "sur", "devant", "et", "puis", "ensuite"}


LOCATION_WORDS = {"sur", "sous", "devant", "derriere", "dans", "au", "par", "pres", "a", "et", "puis", "ensuite", "va", "vas", "avec"}


def _strip_articles(text: str) -> str:
    """Garde le nom de l'objet : coupe au premier mot de lieu/liaison, retire les articles."""
    kept = []
    for w in text.split():
        if w in LOCATION_WORDS:
            break
        if w not in NOT_OBJECT_WORDS:
            kept.append(w)
    return " ".join(kept).strip()


GRAB_ONLY_VERBS = r"(?:attrape|attraper|ramasse|ramasser|prends|prendre|prend|saisis|saisi|saisir|recupere|recuperer|apporte|apporter)"
# Mots qui ne designent pas un objet (pronoms, lieux, deictiques) : "saisis devant toi" n'est pas une cible
GRAB_ONLY_STOP = NOT_OBJECT_WORDS | {"toi", "moi", "devant", "ici", "la", "ca", "cela", "ceci", "ton", "ta", "tes", "mon", "ma", "mes"}


def check_grab_only_intent(query: str):
    """
    Detecte une mission de PRISE SEULE (sans depot) : 'saisis la fleche', 'ramasse la balle',
    'prends la flechette devant toi', 'attrape le cube'.
    Verbe de prise + objet, sans destination. Local, avant toute requete Gemini.
    Retourne {"type": "grab_only", "target": str, "destination": "none", "destination_kind": "none"} ou None.
    Note : 'attrape' / 'ferme' seuls (sans objet) restent des reflexes 'fermer la pince' (match_single_reflex).
    """
    norm = normalize_text(query)
    if not norm:
        return None
    # "au sol", "sur le sol", "par terre" = lieu ou se trouve l'objet, pas une destination
    norm = re.sub(r"\b(?:sur le|sur la|au|par) (?:sol|plancher|terre)\b", " ", norm)
    norm = re.sub(r"\s+", " ", norm).strip()
    # Une phrase avec destination est une mission complete : laissee a check_autonomous_mission_intent
    if re.search(r"\b(depose|deposer|mets|mettre|range|ranger|donne|donner|dans|sur|dedans)\b", norm):
        return None
    m = re.search(rf"\b{GRAB_ONLY_VERBS}\s+(?P<obj>.+)$", norm)
    if not m:
        return None
    # Ignore les articles/determinants, coupe au premier mot de lieu/liaison ("devant", "et", "puis"...)
    words = []
    for w in m.group("obj").split():
        if w in LOCATION_WORDS:
            break
        if w in GRAB_ONLY_STOP:
            continue
        words.append(w)
    target = " ".join(words).strip()
    if not target:
        return None
    return {"type": "grab_only", "target": target, "destination": "none", "destination_kind": "none"}


# --- MODE REFLEXE SANS CLOUD ---
# Utilise quand Gemini ne repond pas : on cherche des mots-cles connus dans la phrase.
# Seul un verbe d'action connu declenche une action (jamais un mouvement sur une phrase vague).
_FALLBACK_KEYWORDS = [
    # (regex sur le texte normalise, action, phrase vocale)
    (r"\b(stop|arrete|arreter|stoppe|stopper|pause|freine|bloque)\b", {"action": "stop", "duration": 0.0}, "Je m'arrête."),
    (r"\b(recule|reculer|marche arriere|en arriere)\b", {"action": "backward", "duration": 1.5}, "Je recule !"),
    (r"\b(avance|avancer|en avant|roule|tout droit)\b", {"action": "forward", "duration": 1.5}, "J'avance !"),
    # Droite / gauche : uniquement un ordre de rotation (verbe de mouvement ou phrase courte "a droite"),
    # jamais une localisation ("a gauche de la boite") qui doit aller a Gemini
    (r"^(?:tourne|vire|pivote|va)?\s*(?:un peu\s+)?(?:a|vers la|sur la)?\s*droite$|^(?:tourne|vire|pivote)\s+(?:un peu\s+)?(?:a|vers la)?\s*droite\b", {"action": "right", "duration": 0.7}, "Je tourne à droite."),
    (r"^(?:tourne|vire|pivote|va)?\s*(?:un peu\s+)?(?:a|vers la|sur la)?\s*gauche$|^(?:tourne|vire|pivote)\s+(?:un peu\s+)?(?:a|vers la)?\s*gauche\b", {"action": "left", "duration": 0.7}, "Je tourne à gauche."),
    (r"\b(clign|cligne)\w*\b", {"action": "blink", "duration": 0.5}, "Clin d'œil !"),
    (r"\b(ouvre|lache|relache)\b.*\b(pince|main)\b|\b(lache|relache)\b", {"action": "open_gripper", "duration": 0.5}, "J'ouvre la pince."),
    (r"\b(ferme|serre)\b.*\b(pince|main)\b", {"action": "close_gripper", "duration": 0.5}, "Je ferme la pince."),
    (r"\b(range|replie|repos)\b.*\b(bras)\b|\bbras au repos\b", {"action": "park", "duration": 1.2}, "Je range mon bras."),
    (r"\b(leve|porte)\b.*\b(bras)\b", {"action": "park", "duration": 1.2}, "Je range mon bras."),
    (r"\b(baisse|descends)\b.*\b(bras)\b", {"action": "park", "duration": 1.2}, "Je range mon bras."),
    (r"\b(regarde|tete)\b.*\b(sol|bas|terre)\b", {"action": "look_ground", "duration": 0.8}, "Je regarde le sol."),
    (r"\b(regarde|tete)\b.*\b(devant|droit)\b", {"action": "look_forward", "duration": 0.8}, "Je regarde droit devant."),
]


# Garde-fou : une phrase qui pose une question, nie une action ou est longue doit aller a Gemini
_QUESTION_OR_NEGATION = re.compile(
    r"\b(qui|quoi|quel|quelle|quels|quelles|comment|pourquoi|combien|ou|est ce|vois|voit|vu|"
    r"ne|n|pas|jamais|plus|rien|peux tu|pourrais tu|sais tu|dis moi|explique)\b"
)
_MAX_LOCAL_WORDS = 8  # au-dela, la phrase est une conversation : Gemini


def _is_clear_order(norm: str) -> bool:
    """Vrai si la phrase normalisee est un ordre simple et court (pas une question, pas une negation)."""
    if len(norm.split()) > _MAX_LOCAL_WORDS:
        return False
    if _QUESTION_OR_NEGATION.search(norm):
        return False
    return True


def match_keyword_fallback(query: str):
    """
    Decodage local par mots-cles. Utilise AVANT Gemini (reflexe) et en secours si Gemini ne repond pas.
    Retourne (phrase_vocale, [action]) ou None si la phrase n'est pas un ordre local clair.
    Ordre : mission de prise seule > mission complete > mots-cles d'action (ordres courts uniquement).
    """
    norm = normalize_text(query)
    if not norm:
        return None
    if not _is_clear_order(norm):
        return None
    grab = check_grab_only_intent(query)
    if grab:
        return (f"Je cherche {grab['target']} et je la saisis.", [{"action": "grab_only", "target": grab["target"], "duration": 0.0}])
    mission = check_autonomous_mission_intent(query)
    if mission:
        return (f"Je prends {mission['target']} et je la dépose.", [{"action": "pick_and_deposit", "target": mission["target"], "destination": mission["destination"], "destination_kind": mission["destination_kind"], "duration": 0.0}])
    for pattern, act, phrase in _FALLBACK_KEYWORDS:
        if re.search(pattern, norm):
            return (phrase, [dict(act)])
    return None


def check_autonomous_mission_intent(query: str):
    """
    Detecte une mission generique 'prendre X puis le deposer / le donner a Y'.
    X : n'importe quel objet (balle, cube, vetement, fleche, ...).
    Y : un contenant (boite, panier...) ou une personne (ex: Milan).
    Retourne {"type": "pick_and_deposit", "target": str, "destination": str,
              "destination_kind": "container"|"person"} ou None.
    """
    norm = normalize_text(query)
    if not norm:
        return None

    # 1) Prise + remise a une personne : "prends la balle et va la donner a milan"
    m = re.search(rf"\b{GRAB_VERBS}\s+(?P<obj>.+?)\s+(?:et\s+)?(?:va\s+|vas\s+)?(?:la\s+|le\s+|l\s+)?{GIVE_VERBS}\s+(?:la\s+|le\s+|l\s+)?(?:a\s+|au\s+)?(?P<dest>\w+)", norm)
    if m:
        target = _strip_articles(m.group("obj"))
        dest = m.group("dest")
        if target and dest and dest not in NOT_OBJECT_WORDS:
            return {"type": "pick_and_deposit", "target": target, "destination": dest,
                    "destination_kind": "person"}

    # 2) Prise + depot dans un contenant : "ramasse la fleche et depose-la dans la boite"
    m = re.search(rf"\b{GRAB_VERBS}\s+(?P<obj>.+?)\s+(?:et\s+)?(?:va\s+|vas\s+)?{DEPOSIT_VERBS}\s+.*?\b(?:dans|sur|au|a)\s+(?:la\s+|le\s+|l\s+|les\s+)?(?P<dest>[a-z]+)", norm)
    if m:
        target = _strip_articles(m.group("obj"))
        dest = m.group("dest")
        if target:
            return {"type": "pick_and_deposit", "target": target, "destination": dest,
                    "destination_kind": "container"}

    return None

def parse_compound_command(query: str):
    """
    Analyse une commande vocale simple ou composée (ex: 'avance puis tourne à droite').
    Retourne (vocal_response, list_of_actions) ou None si non entièrement résoluble en local.
    """
    norm = normalize_text(query)
    if not norm:
        return None

    # Découpage par connecteurs de séquence en priorité
    # 'plus' est inclus car Vosk/Google STT transcrit souvent 'puis' comme 'plus'
    delimiters = r'\b(?:puis|plus|ensuite|et ensuite|et puis|et apres|apres|\bet\b)\b'
    parts = [p.strip() for p in re.split(delimiters, norm) if p.strip()]

    if len(parts) > 1:
        actions = []
        descriptions = []
        all_matched = True
        for p in parts:
            res = match_single_reflex(p)
            if not res:
                all_matched = False
                break
            desc, act = res
            actions.append(act)
            descriptions.append(desc.rstrip('.!'))
        if all_matched:
            vocal_response = f"Compris ! J'enchaîne : {' puis '.join([d.lower() for d in descriptions])}."
            return (vocal_response, actions)

    # Test direct sur clause unique
    single = match_single_reflex(norm)
    if single:
        vocal_desc, act = single
        return (vocal_desc, [act])

    return None
