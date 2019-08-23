/* (c) https://github.com/MontiCore/monticore */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation;

import org.apache.commons.math3.linear.RealVector;

/**
 * Interface that represents a physical vehicle in 3D space.
 * Any positions and vectors are, if not stated otherwise, expressed in the global coordinate system.
 * All values are expressed in SI units.
 */
public interface IPhysicalVehicle extends PhysicalObject {

    /**
     * Function that returns the wheel radius of the physical vehicle
     * @return Wheel radius of the physical vehicle
     */
    double getWheelRadius();

    /**
     * Function that returns the steering angle of the physical vehicle
     * @return Steering angle of the physical vehicle
     */
    double getSteeringAngle();

    /**
     * Function that returns a copy of the position vector of the center of the front right wheel
     * @return Position vector of the center of the front right wheel
     */
    RealVector getFrontRightWheelGeometryPosition();

    /**
     * Function that returns a copy of the position vector of the center of the front left wheel
     * @return Position vector of the center of the front left wheel
     */
    RealVector getFrontLeftWheelGeometryPosition();

    /**
     * Function that returns a copy of the position vector of the center of the back right wheel
     * @return Position vector of the center of the back right wheel
     */
    RealVector getBackRightWheelGeometryPosition();

    /**
     * Function that returns a copy of the position vector of the center of the back left wheel
     * @return Position vector of the center of the back left wheel
     */
    RealVector getBackLeftWheelGeometryPosition();
}
