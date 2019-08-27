package simulation.environment.util;

public interface IBattery {

    public double charge();

    public double timeToCharge(double aimedPercentage);

    public void discharge();

    public double getBatteryPercentage();

    public double getCurrentCapacity();

    public void setVoltageChargingStation(double voltage);

    public void setAmpereChargingStation(double ampere);

    public void connectToChargingStation();

    public void disconnectFromChargingStation();

    // for testing:
    //public double charge(double VoltageInput, double AmpereInput);

}
