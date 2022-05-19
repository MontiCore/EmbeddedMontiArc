/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.montisim.commons.controller.commons.BusEntry;
import de.rwth.montisim.commons.simulation.Sensor;
import de.rwth.montisim.simulation.environment.util.IBattery;

public class Battery implements BatteryInterface, IBattery {

    private Vehicle vehicle;
    private VehicleActuator throttle;
    private VehicleActuator gear;
    private Sensor velocity_sensor;
    private DummyBattery dummyBattery;

    private static final double defaultAmpere = 1.0;

    private double local_deltaT = 0;

    private ConsumptionMethod preferredConsumptionMethod;

    private double oldKineticEnergy;

    private Boolean ChargingStationConnectionStatus;
    private double VoltageChargingStation;
    private double AmpereChargingStation;


    public Battery(Vehicle v, double bCapacity) throws Exception {
        vehicle = v;
        dummyBattery = new DummyBattery(bCapacity);

        this.preferredConsumptionMethod = ConsumptionMethod.CONSUMPTION_THROTTLE_GEAR;

        setThrottle(vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE).get());
        setGear(vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR).get());
        setVelocity(vehicle.getEEVehicle().getSensorByType(BusEntry.SENSOR_VELOCITY).get());

		/* if (vehicle.getControllerBus().isPresent())
			local_deltaT = (double) vehicle.getControllerBus().get().getData(BusEntry.SIMULATION_DELTA_TIME.toString());
		else */
        local_deltaT = 133;
        oldKineticEnergy = 0;

        ChargingStationConnectionStatus = false;
        setVoltageChargingStation(0);
        setAmpereChargingStation(0);
    }


    public Battery(Vehicle v, double bCapacity, double initialBatteryPercentage) throws Exception {
        vehicle = v;
        dummyBattery = new DummyBattery(bCapacity);
        dummyBattery.setPercentage(initialBatteryPercentage);

        this.preferredConsumptionMethod = ConsumptionMethod.CONSUMPTION_THROTTLE_GEAR;

        setThrottle(vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE).get());
        setGear(vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR).get());
        setVelocity(vehicle.getEEVehicle().getSensorByType(BusEntry.SENSOR_VELOCITY).get());
		/* if(vehicle.getControllerBus().isPresent()) {
			local_deltaT = (double) vehicle.getControllerBus().get().getData(BusEntry.SIMULATION_DELTA_TIME.toString());
		} */
        oldKineticEnergy = 0;

        ChargingStationConnectionStatus = false;
        setVoltageChargingStation(0);
        setAmpereChargingStation(0);
    }

    private void setVelocity(Sensor velocity_sensor) throws Exception {
        this.velocity_sensor = velocity_sensor;
        if (velocity_sensor == null) throw new Exception("No velocity sensor found.");
    }

    public void setConsumptionMethod(ConsumptionMethod method) {
        this.preferredConsumptionMethod = method;
    }

    /*
     * throttle and gear values are used to -abstractly- derive the battery consumption
     *
     * 		gear		-> base value of 1, each increment in gear results in increase of 0.5
     * 					-> then this value is multiplied by a constant (3.0)
     *
     * 						//		VEHICLE_DEFAULT_GEAR_MIN = 0;
                            //	    VEHICLE_DEFAULT_GEAR_MAX = 5;
                            //	    VEHICLE_DEFAULT_GEAR_RATE = 1;
     *
     * 		throttle	-> base value of 1.0, each increment in throttle results in increase of 1.5
     *
     * 						//		VEHICLE_DEFAULT_THROTTLE_POSITION_MIN = 0;
                            //	    VEHICLE_DEFAULT_THROTTLE_POSITION_MAX = 1;
                            //	    VEHICLE_DEFAULT_THROTTLE_POSITION_RATE = 0.2;
     *
     * 		gear * throttle		-> base value for consumption
     */
    private double calculateBatteryConsumption_THROTTLE_GEAR() {

        double tVal = this.throttle.getActuatorValueCurrent();
        double gVal = this.gear.getActuatorValueCurrent();

        double gearChainLength = 1.0 + (gVal - Vehicle.VEHICLE_DEFAULT_GEAR_MIN) * 0.5;
        double gearEffectiveValue = gearChainLength * 3.0;

        double throttleEffectiveValue = 1.0 + (tVal - Vehicle.VEHICLE_DEFAULT_THROTTLE_POSITION_MIN) * 1.5;

        double consumption = gearEffectiveValue * throttleEffectiveValue;

        return consumption;
    }

    /*
     * this is the standard energy formula for moving objects
     * 		Kinetic Energy = mass * velocity^2 /2
     *
     * we need "the work done" in rational numbers to find our consumption
     * therefore, we can subtract new kinetic energy value from old kinetic energy value
     * 		Work = nKE - oKE
     *
     * !! please note that this method assumes that the car does NOT change altitude
     * 		it can be updated to reflect also the change in altitude (potential energy difference, that is)
     */
    private double calculateBatteryConsumption_MASS_VELOCITY() {


        // frictionEffect can be calculated based on surface, if needed
        Double frictionEffect = 20.0;

        Double velocityVal = (Double) velocity_sensor.getValue();
        Double mass = vehicle.getPhysicalVehicle().getMass();

        // calculate the new kinetic energy
        Double newKineticEnergy = velocityVal * velocityVal * mass / 2;

        // calculate the difference, thus, the work done by the car
        Double difference = newKineticEnergy - this.oldKineticEnergy;

        // in case the car gets slower, we do not send the consumption as negative, that would be counter-intuitive
        // 		in a similar manner, we send a small consumption, taking friction into consideration
        if (difference < 0)
            difference = 0.0;

        // update old kinetic energy value with the new one
        this.oldKineticEnergy = newKineticEnergy;

        return difference + frictionEffect;
    }

    public void setPercentage(double aimedPercentage) {
        this.dummyBattery.setPercentage(aimedPercentage);
    }

    public double getBatteryConsumption_test_only() {
        double consumptionValue;
        if (this.preferredConsumptionMethod == ConsumptionMethod.CONSUMPTION_MASS_VELOCITY)
            consumptionValue = this.calculateBatteryConsumption_MASS_VELOCITY();
        else
            consumptionValue = this.calculateBatteryConsumption_THROTTLE_GEAR();

        if (consumptionValue < 100)
            consumptionValue = 100;
        return consumptionValue * local_deltaT / 1000.0;
    }

    public double getCriticalPercentage() {
        return this.dummyBattery.getCriticalPercentage();
    }

    public void discharge() {

        double consumptionValue;

        // based on whether the car is a MassPointPhysicalVehicle or ModelicaPhysicalVehicle
        // the calculation method changes
        if (this.preferredConsumptionMethod == ConsumptionMethod.CONSUMPTION_MASS_VELOCITY)
            consumptionValue = this.calculateBatteryConsumption_MASS_VELOCITY();
        else
            consumptionValue = this.calculateBatteryConsumption_THROTTLE_GEAR();

        // for testing big numbers, anything lower than 100 would be ceiled to 100
        // TODO: remove this if statement after tests
        if (consumptionValue < 100)
            consumptionValue = 100;

        // convert consumption to voltage and ampere, albeit abstractly
        double voltage = consumptionValue / defaultAmpere;
        double ampere = defaultAmpere;

        try {
            this.dummyBattery.discharge(voltage, ampere, local_deltaT);
        }
        /*
         * if discharge is not possible (e.g. low battery OR no charge left in battery)
         * 		dummyBattery throws an exception
         *
         * this exception of dummyBattery is caught and response in given in the Vehicle.java
         * 		(setting throttle or motor actuator target values to zero, effectively)
         */ catch (IllegalArgumentException e) {
            //System.out.println("Failed to discharge the battery");
            throw new IllegalArgumentException("Cannot discharge the battery");
        }
    }

    /*
     * for testing charging process
     * 		DO NOT USE this method for release
     */
    public double charge(double VoltageInput, double AmpereInput) {
        return this.dummyBattery.charge(VoltageInput, AmpereInput, local_deltaT);
    }

    /*
     * the advised & default way of charging
     * 		VoltageChargingStation and AmpereChargingStation information
     * 			has to be set by the connected ChargingStation
     * 		furthermore, ChargingStation has to set ChargingStationConnectionStatus
     * 			to true when connected,
     * 			and to false when the car has left the proximity of the ChargingStation itself
     *
     */
    public double charge() {
        return this.dummyBattery.charge(VoltageChargingStation, AmpereChargingStation, local_deltaT);
    }

    public double getBatteryPercentage() {
        return dummyBattery.getPercentage();
    }

    /*
     * for testing charging process
     * 		DO NOT USE this method for release
     */
    public double timeToCharge(double aimedPercentage, double VoltageInput, double AmpereInput) {
        return this.dummyBattery.timeToCharge(aimedPercentage, VoltageInput, AmpereInput);
    }

    /*
     * provided a percentage that is wanted to be achieved (= aimedPercentage),
     * 		returns the remaining time the battery needs to be
     * 		connected to ChargingStation to achieve the given battery percentage
     */
    public double timeToCharge(double aimedPercentage) {
        return this.dummyBattery.timeToCharge(aimedPercentage, VoltageChargingStation, AmpereChargingStation);
    }

    public double getCurrentCapacity() {
        return dummyBattery.getCapacityCurrent();
    }

    public Vehicle getVehicle() {
        return vehicle;
    }

    public void setVehicle(Vehicle vehicle) {
        this.vehicle = vehicle;
    }

    public VehicleActuator getThrottle() {
        return throttle;
    }

    public void setThrottle(VehicleActuator throttle) {
        this.throttle = throttle;
    }

    public VehicleActuator getGear() {
        return gear;
    }

    public void setGear(VehicleActuator gear) {
        this.gear = gear;
    }

    public Boolean getChargingStationConnectionStatus() {
        return this.ChargingStationConnectionStatus;
    }

    public void connectToChargingStation() {
        ChargingStationConnectionStatus = true;
    }

    public void disconnectFromChargingStation() {
        ChargingStationConnectionStatus = false;
        setVoltageChargingStation(0);
        setAmpereChargingStation(0);
    }

    public double getVoltageChargingStation() {
        return VoltageChargingStation;
    }

    public void setVoltageChargingStation(double voltageChargingStation) {
        VoltageChargingStation = voltageChargingStation;
    }

    public double getAmpereChargingStation() {
        return AmpereChargingStation;
    }

    public void setAmpereChargingStation(double ampereChargingStation) {
        AmpereChargingStation = ampereChargingStation;
    }


    public void set_local_delta_t(double local_deltaT) {
        this.local_deltaT = local_deltaT;
    }
}
