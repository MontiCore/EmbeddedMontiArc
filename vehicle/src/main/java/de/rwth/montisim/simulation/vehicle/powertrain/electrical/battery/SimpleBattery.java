/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery;

import java.time.Duration;

import de.rwth.montisim.commons.utils.Time;

public class SimpleBattery extends Battery {

    public SimpleBattery(BatteryProperties properties) {
        super(properties, properties.capacity);
    }

    @Override
    public void discharge(double energy) {
        this.charge -= energy;
        if (this.charge < 0)
            this.charge = 0;
    }

    @Override
    public void charge(double energy) {
        // TODO charging curves
        this.charge += energy;
        if (this.charge > this.properties.capacity)
            this.charge = this.properties.capacity;
    }

    @Override
    public void charge(double volts, double amperes, double seconds) {
        charge(volts * amperes * seconds);
    }

    @Override
    public double percentage() {
        return charge / properties.capacity * 100;
    }

    @Override
    public void setPercentage(double chargePercentage) {
        this.charge = chargePercentage * 0.01f * properties.capacity;
    }

    @Override
    public Duration timeToCharge(double aimedPercentage, double volts, double amperes) {
        // TODO correct charging curve
        if (percentage() >= aimedPercentage)
            return Duration.ZERO;
        double capacityToBeCharged = properties.capacity * (aimedPercentage / 100.0) - charge;
        double power = volts * amperes;
        double time = capacityToBeCharged / power;
        return Time.durationFromSeconds(time);
    }

    @Override
    public boolean isCritical() {
        // TODO Auto-generated method stub
        return charge < properties.critical_charge * 0.01 * properties.capacity;
    }

}