/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.physicsmodel.masspoint;

/**
 * Enum for types of a VehicleActuator
 */
public enum MassPointType {
    /** Mass point located at the front left of the simulated vehicle */
    MASS_POINT_TYPE_WHEEL_FRONT_LEFT,

    /** Mass point located at the front right of the simulated vehicle */
    MASS_POINT_TYPE_WHEEL_FRONT_RIGHT,

    /** Mass point located at the back left of the simulated vehicle */
    MASS_POINT_TYPE_WHEEL_BACK_LEFT,

    /** Mass point located at the back right of the simulated vehicle */
    MASS_POINT_TYPE_WHEEL_BACK_RIGHT,
}
