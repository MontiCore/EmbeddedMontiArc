/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.autopilots;

public class PID {
    final double P;
    final double I;
    final double D;
    double previous_error;
    double integral;

    public PID(double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;
        reset();
    }

    public void reset() {
        previous_error = 0;
        integral = 0;
    }

    public double compute(double dt, double current, double target) {
        /*
         * error := setpoint − measured_value integral := integral + error × dt
         * derivative := (error − previous_error) / dt output := Kp × error + Ki ×
         * integral + Kd × derivative previous_error := error
         */
        double error = target - current;
        integral += error * dt;
        double derivative = dt > 0 ? (error - previous_error) / dt : 0;
        previous_error = error;
        return P * error + I * integral + D * derivative;
    }
}