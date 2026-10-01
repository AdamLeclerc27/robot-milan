"""
Script interactif de test et calibration pour le bras de Milan 2.
Permet à Milan et Adam de :
- Tester les mouvements fluides entre poses clés
- Tester chaque articulation individuellement
- Tester la pince
- Tester la cinématique inverse XYZ
- Relâcher les moteurs
"""

import sys
import time
from robot_arm import RobotArm
from arm_config import ARM_LIMITS, ARM_POSES


def print_menu():
    print("""
=======================================================
       🤖 CONTRÔLEUR DU BRAS DE MILAN 2 🤖
=======================================================
 [1]  Pose 'PARK' (Plié serré au corps pour rouler)
 [2]  Pose 'GROUND_GRAB' (Au sol : Z=2.2cm, X=33.6cm)
 [3]  Pose 'EYE_LEVEL' (Haute : 50cm devant les yeux)
 [4]  Pose 'FORWARD_REACH' (Tendu droit devant)
 [5]  Pose 'HOME' (Position d'attente)
 [6]  Pose 'CARRY' (Replié avec objet tenu)
 -------------------------------------------------------
 [O]  Ouvrir la pince
 [F]  Fermer la pince
 -------------------------------------------------------
 [M]  Déplacer une articulation manuelle (canal 0 à 5)
 [X]  Déplacement cartésien X, Y, Z (en centimètres)
 [S]  Afficher le statut et les angles actuels
 [OFF] Relâcher les moteurs (couper l'effort de maintien)
 [Q]  Quitter
=======================================================
""")


def main():
    arm = RobotArm()
    print("\nInitialisation en position sécurisée (PARK)...")
    arm.move_to_pose("park", duration=1.5)

    while True:
        try:
            print_menu()
            choice = input("👉 Votre choix : ").strip().lower()

            if choice == 'q':
                print("\nArrêt du script. Relâchement des servos...")
                arm.release_all()
                break

            elif choice == '1':
                arm.move_to_pose("park")
            elif choice == '2':
                arm.move_to_pose("ground_grab")
            elif choice == '3':
                arm.move_to_pose("eye_level")
            elif choice == '4':
                arm.move_to_pose("forward_reach")
            elif choice == '5':
                arm.move_to_pose("home")
            elif choice == '6':
                arm.move_to_pose("carry")

            elif choice == 'o':
                print("Ouverture de la pince...")
                arm.open_gripper()
            elif choice == 'f':
                print("Fermeture de la pince...")
                arm.close_gripper(angle=75)

            elif choice == 's':
                arm.print_status()

            elif choice == 'off':
                arm.release_all()

            elif choice == 'm':
                arm.print_status()
                ch_str = input("Numéro du canal (0 à 5) : ").strip()
                if not ch_str.isdigit() or int(ch_str) not in ARM_LIMITS:
                    print("Canal invalide !")
                    continue
                ch = int(ch_str)
                name = ARM_LIMITS[ch]["name"]
                min_ang = ARM_LIMITS[ch]["min"]
                max_ang = ARM_LIMITS[ch]["max"]

                ang_str = input(f"Angle pour {name} (min={min_ang}°, max={max_ang}°) : ").strip()
                try:
                    target_ang = float(ang_str)
                    arm.move_joints({ch: target_ang}, duration=1.0)
                except ValueError:
                    print("Valeur d'angle invalide.")

            elif choice == 'x':
                print("Exemple : '15 0 10' pour 15 cm devant, 0 cm latéral, 10 cm haut")
                coords = input("Entrez X Y Z (en cm) : ").strip().split()
                if len(coords) != 3:
                    print("Veuillez entrer exactement 3 valeurs séparées par des espaces.")
                    continue
                try:
                    x_m = float(coords[0]) / 100.0
                    y_m = float(coords[1]) / 100.0
                    z_m = float(coords[2]) / 100.0
                    print(f"Calcul IK pour X={coords[0]}cm, Y={coords[1]}cm, Z={coords[2]}cm...")
                    success = arm.move_to_xyz(x_m, y_m, z_m, duration=1.5)
                    if success:
                        print("✅ Déplacement effectué.")
                except ValueError:
                    print("Valeurs numériques invalides.")

            else:
                print("Option non reconnue.")

        except KeyboardInterrupt:
            print("\nInterruption détectée. Relâchement des moteurs...")
            arm.release_all()
            break
        except Exception as e:
            print(f"❌ Erreur inattendue : {e}")

    print("Fin du programme.")


if __name__ == "__main__":
    main()
