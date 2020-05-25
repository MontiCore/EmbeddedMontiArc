/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery;

import java.time.Duration;

import de.rwth.montisim.commons.utils.Time;

public class SimpleBattery extends Battery {


    @Override
    public void init(BatteryProperties properties){
        this.capacity = properties.batteryCapacity;
        this.criticalCharge = properties.batteryCriticalChargePercent*0.01*this.capacity;
        this.charge = this.capacity;
    }

    @Override
    public void discharge(double energy){
        this.charge -= energy;
        if (this.charge < 0) this.charge = 0;
    }

    @Override
    public void charge(double energy){
        // TODO charging curves
        this.charge += energy;
        if (this.charge > this.capacity) this.charge = this.capacity;
    }

    @Override
    public void charge(double volts, double amperes, double seconds){
        charge(volts*amperes*seconds);
    }

    @Override
    public double percentage(){
        return charge/capacity*100;
    }

    @Override
    public void setPercentage(double chargePercentage){
        this.charge = chargePercentage*0.01f*capacity;
    }

    @Override
    public Duration timeToCharge(double aimedPercentage, double volts, double amperes) {
        // TODO correct charging curve
		if (percentage() >= aimedPercentage)
			return Duration.ZERO;
		double capacityToBeCharged = capacity* (aimedPercentage/100.0) - charge;
		double power = volts * amperes;
        double time = capacityToBeCharged / power;
		return Time.durationFromSeconds(time);
	}
}