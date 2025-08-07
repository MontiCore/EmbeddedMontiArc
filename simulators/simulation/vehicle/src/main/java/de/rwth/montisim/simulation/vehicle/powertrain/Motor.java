/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain;

public interface Motor {
    // "Engine" functions, used by the physics model.
    public double getMaxTorque(double rpm); // Goes to zero if no more energy/fuel

    public void consume(double energy, double delta_t); // The energy output consumed from the motor (delta_t in seconds)

    public void regenerate(double energy, double delta_t); // The energy input to the braking system (delta_t in seconds)
}