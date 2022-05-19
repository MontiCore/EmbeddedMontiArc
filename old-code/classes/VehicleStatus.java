/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

/**
 * State of vehicle parts
 */
public enum VehicleStatus {
    /** Status is OK. No problems sensed. */
    VEHICLE_STATUS_OK(1000),

    /** Status is OK, but component should be inspected in workshop */
    VEHICLE_STATUS_SERVICE_REQUIRED(2000),

    /** Component damaged, but still working */
    VEHICLE_STATUS_DAMAGED(3000),

    /** Failure of the component imminent */
    VEHICLE_STATUS_CRITICAL(4000),

    /** Component is broken */
    VEHICLE_STATUS_FAILURE(5000);

}
