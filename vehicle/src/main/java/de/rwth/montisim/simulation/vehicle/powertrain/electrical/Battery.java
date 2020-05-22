package de.rwth.montisim.simulation.vehicle.powertrain.electrical;

import java.time.Duration;

import de.rwth.montisim.simulation.vehicle.vehicleproperties.ElectricalPTProperties;

public abstract class Battery {
    public double capacity;
    public double charge;
    public double criticalCharge;
    
    abstract public void init(ElectricalPTProperties properties);
    abstract public void discharge(double energy);
    abstract public void charge(double energy);
    abstract public void charge(double volts, double amperes, double seconds);
    abstract public double percentage();
    abstract public void setPercentage(double chargePercentage);
    abstract public Duration timeToCharge(double aimedPercentage, double volts, double amperes);
}