"""
Localisation generique d'une cible dans l'image (objet, contenant ou personne).

- Chemin rapide local : detecteurs couleur (fleche bleue, boite) deja valides sur le robot.
- Chemin generique : Gemini (REST, meme API que app.py) renvoie une boite englobante
  pour n'importe quel objet demande (balle, cube, vetement...).
Coordonnees retournees : cx, cy, dans 0..1000 ; box_2d = [y0, x0, y1, x1] dans 0..1000.
"""
import base64
import json
import time
import unicodedata

import cv2
import requests

GEMINI_URL = "https://generativelanguage.googleapis.com/v1beta/interactions"
GEMINI_MODELS = [("gemini-3.5-flash-lite", 6), ("gemini-3-flash-preview", 12)]


def _norm(text):
    """Minuscules sans accents, pour comparer les noms de cibles."""
    text = unicodedata.normalize("NFKD", text.lower())
    return "".join(c for c in text if not unicodedata.combining(c))


def _box_to_result(box_2d, frame_shape):
    y0, x0, y1, x1 = [max(0, min(1000, int(v))) for v in box_2d]
    if y1 <= y0 or x1 <= x0:
        return None
    return {
        "found": True,
        "cx": (x0 + x1) // 2,
        "cy": (y0 + y1) // 2,
        "area": float((x1 - x0) * (y1 - y0)) / 1000000.0 * frame_shape[0] * frame_shape[1],
        "box_2d": [y0, x0, y1, x1],
    }


def _frame_to_b64(frame):
    small = cv2.resize(frame, (480, int(480 * frame.shape[0] / frame.shape[1])))
    ok, buf = cv2.imencode('.jpg', small, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    return base64.b64encode(buf.tobytes()).decode('utf-8') if ok else None


def _ask_gemini_box(frame, label, api_key):
    """Demande a Gemini la boite englobante de 'label' dans l'image. Retourne dict ou None."""
    if not api_key or frame is None:
        return None
    b64 = _frame_to_b64(frame)
    if not b64:
        return None
    system_prompt = (
        "Tu localises un objet dans une image. Reponds UNIQUEMENT avec un JSON valide :\n"
        '{"found": true/false, "box_2d": [y0, x0, y1, x1]}\n'
        "Coordonnees entieres entre 0 et 1000 (0 = haut/gauche, 1000 = bas/droite). "
        "Si l'objet demande n'est pas visible, mets found=false et box_2d=[0,0,0,0]."
    )
    payload = {
        "model": None,
        "system_instruction": system_prompt,
        "input": [
            {"type": "image", "mime_type": "image/jpeg", "data": b64},
            {"type": "text", "text": f"Localise : {label}"},
        ],
        "response_format": [{"type": "text", "mime_type": "application/json"}],
    }
    headers = {"x-goog-api-key": api_key, "Content-Type": "application/json"}
    data = None
    for model_name, timeout_s in GEMINI_MODELS:
        payload["model"] = model_name
        try:
            t0 = time.time()
            res = requests.post(GEMINI_URL, headers=headers, json=payload, timeout=timeout_s)
            res.raise_for_status()
            data = res.json()
            print(f"[Cible] Gemini {model_name} : {time.time() - t0:.2f} s", flush=True)
            break
        except Exception as err:
            print(f"[Cible] Erreur Gemini {model_name} : {err}", flush=True)
    if not data:
        return None
    try:
        text = ""
        for step in data.get("steps", []):
            if step.get("type") == "model_output":
                for content in step.get("content", []):
                    if content.get("type") == "text":
                        text += content.get("text", "")
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            parsed = json.loads(text.replace("```json", "").replace("```", "").strip())
    except Exception as err:
        print(f"[Cible] Reponse Gemini illisible : {err}", flush=True)
        return None
    if not parsed.get("found"):
        return None
    return _box_to_result(parsed.get("box_2d", [0, 0, 0, 0]), frame.shape)


class TargetLocator:
    """Trouve une cible generique. local_detectors : dict {mot-cle: fonction(frame) -> dict|None}."""

    def __init__(self, api_key=None, local_detectors=None):
        self.api_key = api_key
        self.local_detectors = local_detectors or {}

    def has_local(self, label):
        """Vrai si un detecteur couleur connait cette cible (sinon Gemini est utilise)."""
        key = _norm(label)
        return any(k in key for k in self.local_detectors)

    def find(self, frame, label):
        """Retourne un dict {found, cx, cy, area, box_2d} ou None."""
        if frame is None:
            return None
        key = _norm(label)
        for keyword, detector in self.local_detectors.items():
            if keyword in key:
                res = detector(frame)
                if res and res.get("found"):
                    return res
        return _ask_gemini_box(frame, label, self.api_key)
