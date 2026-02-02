class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        
        # Mémoire pour l'intégrale et la dérivée
        self.integral = 0
        self.prev_error = 0

    def compute(self, target, current):
        """
        Calcule la commande u en fonction de la cible et de la valeur actuelle.
        """
        error = target - current
        
        # Terme Proportionnel
        P = self.Kp * error
        
        # Terme Intégral (somme des erreurs)
        self.integral += error * self.dt
        I = self.Ki * self.integral
        
        # Terme Dérivé (variation de l'erreur)
        # Note : error - prev_error est l'inverse de la vitesse
        derivative = (error - self.prev_error) / self.dt
        D = self.Kd * derivative
        
        # Mise à jour pour le tour suivant
        self.prev_error = error
        
        return P + I + D