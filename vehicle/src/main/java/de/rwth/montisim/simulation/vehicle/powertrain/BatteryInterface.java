/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain;

public interface BatteryInterface {
	
	//public void setConsumptionMethod(Battery.consumptionMethod method);
	
	public double charge();
	
	public double timeToCharge(double aimedPercentage);
	
	public void discharge();
	
	public double getBatteryPercentage();
	
	public double getCurrentCapacity();
	
	// for testing:
	public double charge(double VoltageInput, double AmpereInput);
	
	public double timeToCharge(double aimedPercentage, double VoltageInput, double AmpereInput);
}
