/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
