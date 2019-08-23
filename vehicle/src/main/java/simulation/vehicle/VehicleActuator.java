/* (c) https://github.com/MontiCore/monticore */
package simulation.vehicle;

/**
 * Class that represents an actuator of the vehicle
 */
public class VehicleActuator {

    /** Type of the actuator */
    private VehicleActuatorType actuatorType;

    /** Minimum value possible to set */
    private double actuatorValueMin;

    /** Maximum value possible to set */
    private double actuatorValueMax;

    /** Current value */
    private double actuatorValueCurrent;

    /** Value that should be set */
    private double actuatorValueTarget;

    /** Change rate of the actuator*/
    private double actuatorChangeRate;

    /**
     * Constructor for the actuator class, only sets the type, minimum, maximum and change rate values
     * Current value and target value are initialised to 0.0 or the given minimum value if 0.0 is not in the given range
     *
     * @param actuatorType Type of the actuator
     * @param actuatorValueMin Minimum allowed value of the actuator
     * @param actuatorValueMax Maximum allowed value of the actuator
     * @param actuatorChangeRate Change rate of the actuator
     */
    public VehicleActuator(VehicleActuatorType actuatorType, double actuatorValueMin, double actuatorValueMax, double actuatorChangeRate){
        if(actuatorValueMin > actuatorValueMax){
            throw new IllegalArgumentException("Lower end " + actuatorValueMin + " should not be higher than the upper end " + actuatorValueMax + ".");
        }
        if(actuatorChangeRate < 0){
            throw new IllegalArgumentException("Change rate " + actuatorChangeRate + " should not be negative.");
        }
        this.actuatorType = actuatorType;
        this.actuatorValueMin = actuatorValueMin;
        this.actuatorValueMax = actuatorValueMax;
        this.actuatorChangeRate = actuatorChangeRate;
        if(actuatorValueMin <= 0.0 && 0.0 <= actuatorValueMax){
            actuatorValueTarget = 0.0;
            actuatorValueCurrent = 0.0;
        }else {
            actuatorValueCurrent = actuatorValueMin;
            actuatorValueTarget = actuatorValueMin;
        }
    }

    /**
     * Function that computes an update tick for the actuator
     *
     * @param timeDiff Time difference considered in the update measured in seconds
     */
    protected void update(double timeDiff) {
        // Total change of value in given time span
        double valueDiff = (actuatorChangeRate * timeDiff);

        if (Math.abs(actuatorValueTarget - actuatorValueCurrent) <= valueDiff) {
            // If the difference between current value and target value is less or equal valueDiff, then just set target value
            actuatorValueCurrent = actuatorValueTarget;
        }else {
            // Otherwise add or subtract valueDiff to current value
            actuatorValueCurrent = actuatorValueCurrent + ((actuatorValueCurrent < actuatorValueTarget) ? valueDiff : -valueDiff);
        }
    }

    /**
     * Getter for the type of the actuator
     *
     * @return Type of the actuator
     */
    public VehicleActuatorType getActuatorType() {
        return actuatorType;
    }

    /**
     * Getter for the minimum allowed value of the actuator
     *
     * @return Minimum allowed value of the actuator
     */
    public double getActuatorValueMin() {
        return actuatorValueMin;
    }

    /**
     * Getter for the maximum allowed value of the actuator
     *
     * @return Maximum allowed value of the actuator
     */
    public double getActuatorValueMax() {
        return actuatorValueMax;
    }

    /**
     * Getter for the current value of the actuator
     *
     * @return Current value of the actuator
     */
    public double getActuatorValueCurrent() {
        return actuatorValueCurrent;
    }

    /**
     * Getter for the target value of the actuator
     * Actuator will change to target value in time (limited by change rate)
     *
     * @return Target value of the actuator
     */
    public double getActuatorValueTarget() {
        return actuatorValueTarget;
    }

    /**
     * Setter for the target value of the actuator
     * Actuator will change to target value in time (limited by change rate)
     *
     * @param value New target value of the actuator
     */
    public void setActuatorValueTarget(double value){
        if (value > actuatorValueMax || value < actuatorValueMin || Double.isNaN(value)) {
            throw new IllegalArgumentException("VehicleActuator: setActuatorValueTarget - Invalid actuator target value: " + value + " for actuator: " + this);
        }
        actuatorValueTarget = value;
    }

    /**
     * Setter for the current value of the actuator
     * Used to reset actuators in a collision
     *
     * @param value New value of the actuator
     */
    public void setActuatorValueCurrent(double value){
        if (value > actuatorValueMax || value < actuatorValueMin || Double.isNaN(value)) {
            throw new IllegalArgumentException("VehicleActuator: setActuatorValueCurrent - Invalid actuator target value: " + value + " for actuator: " + this);
        }
        actuatorValueCurrent = value;
    }

    /**
     * Getter for the change rate of the actuator
     *
     * @return Change rate of the actuator
     */
    public double getActuatorValueChangeRate() {
        return actuatorChangeRate;
    }

    /**
     * Overwrite toString() to get a nice output for vehicles
     * @return String that contains all information of vehicles
     */
    @Override
    public String toString() {
        return  "VehicleActuator " + hashCode() + ": actuatorType: " + actuatorType +
                " , actuatorValueMin: " + actuatorValueMin +
                " , actuatorValueMax: " + actuatorValueMax +
                " , actuatorValueCurrent: " + actuatorValueCurrent +
                " , actuatorValueTarget: " + actuatorValueTarget +
                " , actuatorChangeRate: " + actuatorChangeRate;
    }
}
