/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.util;

public interface IBattery {

    public enum ConsumptionMethod {
        CONSUMPTION_MASS_VELOCITY,
        CONSUMPTION_THROTTLE_GEAR,
    }

    public void setConsumptionMethod(ConsumptionMethod method);

    public double charge();

    public double timeToCharge(double aimedPercentage);

    public void discharge();

    public double getBatteryPercentage();

    public double getCurrentCapacity();

    public double getVoltageChargingStation();

    public void setVoltageChargingStation(double voltage);

    public double getAmpereChargingStation();

    public void setAmpereChargingStation(double ampere);

    public void connectToChargingStation();

    public void disconnectFromChargingStation();

    public Boolean getChargingStationConnectionStatus();

    // for testing:
    //public double charge(double VoltageInput, double AmpereInput);

    //public double timeToCharge(double aimedPercentage, double VoltageInput, double AmpereInput);
}
