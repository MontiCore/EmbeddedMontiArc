/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery;

import java.time.Duration;

public abstract class Battery {
    final transient BatteryProperties properties;
    public double charge;

    public Battery(BatteryProperties properties, double charge) {
        this.properties = properties;
        this.charge = charge;
    }

    abstract public boolean isCritical();

    abstract public void discharge(double energy);

    abstract public void charge(double energy);

    abstract public void charge(double volts, double amperes, double seconds);

    abstract public double percentage();

    abstract public void setPercentage(double chargePercentage);

    abstract public Duration timeToCharge(double aimedPercentage, double volts, double amperes);

}