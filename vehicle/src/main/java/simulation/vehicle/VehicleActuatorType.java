/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

/**
 * Enum for types of a VehicleActuator
 */
public enum VehicleActuatorType {
    /** The motor of the vehicle */
    VEHICLE_ACTUATOR_TYPE_MOTOR,

    /** The brakes of the vehicle */
    VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT,
    VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT,
    VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT,
    VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT,

    /** A motor turning the steering wheel of the vehicle */
    VEHICLE_ACTUATOR_TYPE_STEERING,

    /**All the pedals inside the car*/
    VEHICLE_ACTUATOR_TYPE_BRAKE,

    VEHICLE_ACTUATOR_TYPE_THROTTLE,

    VEHICLE_ACTUATOR_TYPE_CLUTCH,

    VEHICLE_ACTUATOR_TYPE_GEAR,
}
