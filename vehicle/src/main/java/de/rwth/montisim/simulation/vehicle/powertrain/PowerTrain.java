/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain;


public abstract class PowerTrain {
    public final String type;
    public Motor motor;

    public PowerTrain(String type) {
        this.type = type;
    }

    public abstract double getFuelPercentage();
    /**
     * @return The current ratio between the motor & the wheels (ratio = motor revolutions / wheel revolutions)
     */
    public abstract double getTransmissionRatio();
}