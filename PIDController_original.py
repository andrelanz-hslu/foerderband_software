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

        # PID constants:
        self.kp = 180 / 1023.0 * 36
        self.Tn = 20
        self.Tv = 0

    def reset(self):
        """
        Restore controller with initial values.
        """
        self.error_linear = 0
        self.error_integral = 0

    def calculate_controller_output(self, actual_value):

        """
        Calculate next target values with the help of a PID controller.
        """
        #  1. Speichern Sie den vorherigen Fehler in der Variablen
        #     'error_linear_old', berechnen Sie den neuen Fehler und
        #     speichern Sie diesen in self.error_linear
        # TODO: Implementieren
        #  2. Berechnen Sie
        #     - den aktuellen Positions-Fehler 'self.error_linear'
        #     - das aktuelle Fehler-Integral 'self.error_integral'; denken
        #       Sie dabei an windup
        #     - das aktuelle Fehler-Derivative 'error_derivative'
        # TODO: Implementieren
        #  3. Berechnen Sie aus den Fehlern die P, I und D-Anteile;
        #     Sie können diese Werte in den Variablen p_part, i_part
        #     und d_part abspeichern oder die Berechnungen direkt in die
        #     Liste der pid_actions schreiben
        p_part = 0
        i_part = 0
        d_part = 0
        # TODO: Implementieren

        # Save the three parts of the controller in a vector
        pid_actions = [p_part, i_part, d_part]
        # The output speed is the sum of the parts, 1023 equals 5V = max output
        controller_output = max(min(sum(pid_actions), self.anti_windup), -self.anti_windup)

        return int(controller_output), pid_actions
