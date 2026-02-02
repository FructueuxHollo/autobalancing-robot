import numpy as np
import time
import sys
from robot import Robot
from controller import PID
from visualizer import Visualizer
import config

# --- PARAMÈTRES DES SCÉNARIOS ---

def run_simulation(nom_scenario, gains_angle, gains_pos, etat_initial, duree=5.0):
    """
    Fonction générique pour lancer une démo.
    gains_angle : (Kp, Ki, Kd)
    gains_pos   : (Kp, Ki, Kd)
    """
    print(f"\n--- Lancement du Scénario : {nom_scenario} ---")
    print(f"Gains Angle : {gains_angle}")
    print(f"Gains Pos   : {gains_pos}")
    print("Initialisation...")
    time.sleep(1) # Petite pause pour le suspense

    # 1. Configuration
    bot = Robot()
    
    # Contrôleurs forcés avec les paramètres du scénario
    pid_angle = PID(gains_angle[0], gains_angle[1], gains_angle[2], config.dt)
    pid_pos   = PID(gains_pos[0], gains_pos[1], gains_pos[2], config.dt)
    
    state = np.array(etat_initial)
    history = {'time': [], 'theta': [], 'x': [], 'u': [], 'target_theta': []}
    steps = int(duree / config.dt)

    print("Simulation en cours...")

    # 2. Boucle de Simulation
    for i in range(steps):
        x = state[0]
        theta = state[2]
        
        # --- Stratégie de Contrôle (Cascade) ---
        
        # Si les gains de position sont nuls, on désactive cette boucle
        target_theta = 0.0
        if gains_pos[0] != 0 or gains_pos[2] != 0:
            target_theta = pid_pos.compute(target=0.0, current=x)
            target_theta = np.clip(target_theta, -0.2, 0.2) # Max 11 degrés
        
        # Si les gains d'angle sont nuls, u restera à 0 (Scenario 1)
        u = 0.0
        if gains_angle[0] != 0 or gains_angle[2] != 0:
            u = -pid_angle.compute(target=target_theta, current=theta)
        
        # Saturation et Physique
        u = np.clip(u, -24, 24)
        state = bot.step(state, u, config.dt)
        
        # Enregistrement
        history['time'].append(i * config.dt)
        history['theta'].append(state[2])
        history['x'].append(state[0])
        history['u'].append(u)
        history['target_theta'].append(target_theta)
        
        # Gestion du Crash (Arrêt anticipé de la simulation mais on garde l'historique)
        if abs(state[2]) > np.pi/3:
            print(f"💥 CRASH du robot à t = {i*config.dt:.2f}s !")
            # On remplit le reste de l'historique avec la dernière valeur pour que l'anim ne plante pas
            remaining = steps - i - 1
            for _ in range(remaining):
                history['time'].append(history['time'][-1] + config.dt)
                history['theta'].append(state[2]) # Reste au sol
                history['x'].append(state[0])
                history['u'].append(0)
                history['target_theta'].append(target_theta)
            break

    # 3. Lancement de la visualisation
    print("Génération de l'animation... (Fermez la fenêtre pour quitter)")
    visu = Visualizer(history)
    # On surcharge le titre de la fenêtre pour la démo
    visu.animate()

def menu():
    while True:
        print("\n=============================================")
        print("   DÉMO ROBOT AUTO-BALANCEUR - SÉLECTION")
        print("=============================================")
        print("1. Scénario 'Chute Libre' (Pas de contrôle)")
        print("2. Scénario 'Parkinson' (Contrôle inadapté/Instable)")
        print("3. Scénario 'Idéal' (Retour à la base stable)")
        print("4. Quitter")
        
        choix = input("\nVotre choix (1-4) : ")
        
        if choix == '1':
            # SCÉNARIO 1 : CHUTE LIBRE
            # Conditions : Gains à 0, petit angle initial
            run_simulation(
                nom_scenario="Sans Contrôle (Gravité seule)",
                gains_angle=(0.0, 0.0, 0.0), # Pas de réponse
                gains_pos=(0.0, 0.0, 0.0),
                etat_initial=[0.0, 0.0, 0.1, 0.0], # x=0, theta=0.1 rad
                duree=10.0
            )
            
        elif choix == '2':
            # SCÉNARIO 2 : CONTRÔLE INADAPTÉ
            # Conditions : Kp très fort (réponse violente), Kd nul (pas d'amortissement)
            # Le robot va osciller de plus en plus fort jusqu'au crash
            run_simulation(
                nom_scenario="Contrôle Instable (Oscillations)",
                gains_angle=(3.0, 0.0, 0.05), # Kp trop fort, Kd=0 (Mortel)
                gains_pos=(0.05, 0.0, 0.1),    # Pas de gestion de position
                etat_initial=[0.0, 0.0, 0.5, 0.0], # Petit angle départ
                duree=10.0
            )
            
        elif choix == '3':
            # SCÉNARIO 3 : IDÉAL
            # Conditions : Gains optimaux (ceux que tu as trouvés ou des valeurs sûres)
            # Départ loin (x=-1.5m), doit revenir à 0 en douceur.
            run_simulation(
                nom_scenario="Contrôle Optimal (Retour Base)",
                gains_angle=(2.0, 0.0, 0.1725),   # Tes valeurs PID Angle
                gains_pos=(0.05, 0.0, 0.1),     # Tes valeurs PID Position
                etat_initial=[-1.0, 0.0, 1.0, 0.0], # Départ à -1.5m
                duree=15.0
            )
            
        elif choix == '4':
            print("Fin de la démo.")
            sys.exit()
        else:
            print("Choix invalide.")

if __name__ == "__main__":
    try:
        menu()
    except KeyboardInterrupt:
        print("\nArrêt forcé.")