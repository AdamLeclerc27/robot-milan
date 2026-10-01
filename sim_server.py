"""
Serveur de simulation 3D pour Milan 2.
Permet d'ouvrir le visualiseur 3D dans le navigateur tout en exécutant
des mouvements simulés en Python pour voir le bras bouger en direct.
"""

import http.server
import socketserver
import webbrowser
import threading
import time
import os
from robot_arm import RobotArm

PORT = 8080


def start_server():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    handler = http.server.SimpleHTTPRequestHandler
    # Supprimer les logs excessifs d'accès HTTP
    handler.log_message = lambda *args: None

    with socketserver.TCPServer(("", PORT), handler) as httpd:
        print(f"[Serveur 3D] 🌐 Serveur lancé sur http://localhost:{PORT}/arm_visualizer.html")
        httpd.serve_forever()


def run_demo_routine(arm: RobotArm):
    """Enchaîne une petite démo automatique pour observer les mouvements en 3D."""
    print("\n[Démo 3D] Démarrage de la séquence de démonstration...")
    time.sleep(1.0)
    
    poses_demo = [
        ("home", "Retour à la position neutre (HOME)"),
        ("observe", "Déploiement en observation haute (OBSERVE)"),
        ("forward_reach", "Tente d'atteindre un objet devant (FORWARD REACH)"),
        ("ground_grab", "Descente au niveau du sol (GROUND GRAB)"),
        ("carry", "Repli avec objet sécurisé (CARRY)"),
        ("park", "Rangement compact pour rouler (PARK)")
    ]

    for pose_name, description in poses_demo:
        print(f" -> {description}...")
        arm.move_to_pose(pose_name, duration=1.5)
        time.sleep(1.0)

    print("[Démo 3D] ✅ Démonstration terminée !\n")


def main():
    # 1. Démarrer le serveur HTTP en arrière-plan
    server_thread = threading.Thread(target=start_server, daemon=True)
    server_thread.start()

    time.sleep(0.5)
    url = f"http://localhost:{PORT}/arm_visualizer.html"
    print(f"\nOuverture du visualiseur 3D dans votre navigateur : {url}")
    try:
        webbrowser.open(url)
    except Exception:
        pass

    # 2. Initialiser le bras en simulation
    arm = RobotArm(simulation=True)

    print("\n=======================================================")
    print("   🎮 SIMULATION 3D CONNECTÉE AU CODE PYTHON 🎮")
    print("=======================================================")
    print(" [1] Lancer la routine de démo automatique")
    print(" [2] Tester la pose 'PARK'")
    print(" [3] Tester la pose 'GROUND_GRAB'")
    print(" [4] Tester la pose 'HOME'")
    print(" [5] Ouvrir / Fermer la pince")
    print(" [Q] Quitter")
    print("=======================================================\n")

    while True:
        try:
            choice = input("👉 Choix simulation : ").strip().lower()
            if choice == 'q':
                break
            elif choice == '1':
                run_demo_routine(arm)
            elif choice == '2':
                arm.move_to_pose('park')
            elif choice == '3':
                arm.move_to_pose('ground_grab')
            elif choice == '4':
                arm.move_to_pose('home')
            elif choice == '5':
                print("Ouverture pince...")
                arm.open_gripper()
                time.sleep(0.8)
                print("Fermeture pince...")
                arm.close_gripper(75)
        except KeyboardInterrupt:
            break

    print("Fin de la simulation.")


if __name__ == "__main__":
    main()
