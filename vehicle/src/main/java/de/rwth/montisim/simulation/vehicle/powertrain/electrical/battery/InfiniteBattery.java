/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.electrical.battery;

import java.time.Duration;

public class InfiniteBattery extends Battery {
    @Override
    public void init(BatteryProperties properties){
        this.charge = Float.POSITIVE_INFINITY;
        this.capacity = Float.POSITIVE_INFINITY;
        this.criticalCharge = 0;
    }

    @Override
    public void discharge(double energy){}

    @Override
    public void charge(double energy){}

    @Override
    public void charge(double volts, double amperes, double seconds){}

    @Override
    public double percentage(){
        return 100;
    }

    @Override
    public void setPercentage(double p) {}

    @Override
    public Duration timeToCharge(double aimedPercentage, double volts, double amperes) {
		return Duration.ZERO;
	}
}