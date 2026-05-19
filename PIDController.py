# Copyright 2020 Hochschule Luzern - Informatik
# Author: Simon van Hemert <simon.vanhemert@hslu.ch>
# Author: Peter Sollberger <peter.sollberger@hslu.ch>

class PIDController:
    """
    Implements a PID controller.
    """

    def __init__(self):
        # Initialize variables
        self.reference_value = 415  # Reference (e.g. position in mm)
        self.error_linear = self.reference_value  # Initial error
        self.error_integral = 0
        self.anti_windup = 1023  # Anti-windup for Integrator, 1023 equals 5V = max speed

        # PID constants - können direkt gesetzt oder über Tn/Tv berechnet werden
        self.kp = 180 / 1023.0
        
        # Alternative: Berechnung über Tn und Tv
        self.Tn = 20
        self.Tv = 0
        
        self.error_linear_old = self.reference_value  # Store previous error (initialized same as error_linear)

    def reset(self):
        """
        Restore controller with initial values.
        """
        self.error_linear = 0
        self.error_integral = 0
        self.error_linear_old = 0

    def calculate_controller_output(self, actual_value):

        """
        Calculate next target values with the help of a PID controller.
        """
        # Berechne ki und kd basierend auf Tn und Tv
        ki = self.kp / self.Tn

        kd = self.kp * self.Tv

        # 7. Speicherwert - Speichere den aktuellen Fehler für die nächste Iteration
        self.error_linear_old = self.error_linear

        # 1. Fehler e(t) - Berechne den aktuellen Positionsfehler
        self.error_linear = self.reference_value - actual_value

        # 2. Integral - Berechne das aktuelle Fehler-Integral
        self.error_integral = self.error_integral + self.error_linear * 0.01
        
        # Anti-Windup: Begrenzte das Integral, damit ki * error_integral die maximale Ausgabe nicht überschreitet
        if ki != 0:
            integral_limit = self.anti_windup / ki
            self.error_integral = max(min(self.error_integral, integral_limit), -integral_limit)

        # 3. Ableitung - Berechne die Fehlerableitung (Derivative)
        error_derivative = (self.error_linear - self.error_linear_old) / 0.01

        # 4. P-Anteil
        p_part = self.kp * self.error_linear

        # 5. I-Anteil
        i_part = ki * self.error_integral

        # 6. D-Anteil
        d_part = kd * error_derivative

        # 8. Stellgrösse u(t) - Summe der Anteile
        u = p_part + i_part + d_part

        # Save the three parts of the controller in a vector
        pid_actions = [p_part, i_part, d_part]
        # The output speed is the sum of the parts, 1023 equals 5V = max output
        controller_output = max(min(u, self.anti_windup), -self.anti_windup)

        return int(controller_output), pid_actions
