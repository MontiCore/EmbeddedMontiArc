/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery;

import java.time.Duration;

public abstract class Battery {
    public double capacity;
    public double charge;
    public double criticalCharge;
    
    abstract public void init(BatteryProperties properties);
    abstract public void discharge(double energy);
    abstract public void charge(double energy);
    abstract public void charge(double volts, double amperes, double seconds);
    abstract public double percentage();
    abstract public void setPercentage(double chargePercentage);
    abstract public Duration timeToCharge(double aimedPercentage, double volts, double amperes);
}