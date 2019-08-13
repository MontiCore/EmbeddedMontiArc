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

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import java.util.List;
import java.util.Map;

/**
 * Interface that represents a box shaped physical object in a 3D space, that can be rotated.
 * Any positions and vectors are, if not stated otherwise, expressed in the global coordinate system.
 * All values are, if not stated otherwise, expressed in SI units.
 */
public interface PhysicalObject {

    /**
     * Function that returns a copy of the center of mass position vector
     * @return Position vector of the center of mass
     */
    RealVector getPosition();

    /**
     * Function that sets the center of mass position vector
     * @param position New position vector of the center of mass
     */
    void setPosition(RealVector position);

    /**
     * Function that returns a copy of the rotation matrix around the center of mass
     * @return Rotation matrix around the center of mass
     */
    RealMatrix getRotation();

    /**
     * Function that sets the rotation matrix around the center of mass
     * @param rotation New rotation matrix around the center of mass
     */
    void setRotation(RealMatrix rotation);

    /**
     * Function that returns a copy of the velocity vector of the center of mass
     * @return Velocity vector of the center of mass
     */
    RealVector getVelocity();

    /**
     * Function that sets the velocity vector of the center of mass
     * @param velocity New velocity vector of the center of mass
     */
    void setVelocity(RealVector velocity);

    /**
     * Function that returns a copy of the angular velocity vector around the center of mass
     * @return Angular velocity vector around the center of mass
     */
    RealVector getAngularVelocity();

    /**
     * Function that sets the angular velocity vector around the center of mass
     * @param angularVelocity New angular velocity around of the center of mass
     */
    void setAngularVelocity(RealVector angularVelocity);

    /**
     * Function that adds an external force acting on the center of mass
     * @param force Force vector that acts on the center of mass
     */
    void addForce(RealVector force);

    /**
     * Function that add an external torque acting around the center of mass
     * @param torque Torque vector that acts around the center of mass
     */
    void addTorque(RealVector torque);

    /**
     * Function that returns the mass of the object
     * @return Mass of the physical object
     */
    double getMass();

    /**
     * Function that sets the mass of the object
     * @param mass New mass of the physical object
     */
    void setMass(double mass);

    /**
     * Function that returns the width of the object
     * @return Width of the object
     */
    double getWidth();

    /**
     * Function that sets the width of the object
     * @param width New width of the object
     */
    void setWidth(double width);

    /**
     * Function that returns the length of the object
     * @return Length of the object
     */
    double getLength();

    /**
     * Function that sets the length of the object
     * @param length New length of the object
     */
    void setLength(double length);

    /**
     * Function that returns the height of the object
     * @return Height of the object
     */
    double getHeight();

    /**
     * Function that sets the height of the object
     * @param height New height of the object
     */
    void setHeight(double height);

    /**
     * Function that returns a copy of center of geometry position vector
     * @return Position vector of the center of geometry
     */
    RealVector getGeometryPosition();

    /**
     * Function that sets the center of geometry position vector.
     * @param geometryPosition New position vector of the center of geometry
     */
    void setGeometryPosition(RealVector geometryPosition);

    /**
     * Function that returns a copy of the vector pointing from the center of mass position to the center of geometry position
     * @return Offset vector of the center of mass position to the center of geometry position
     */
    RealVector getGeometryPositionOffset();

    /**
     * Function that sets the vector pointing from the center of mass position to the center of geometry position
     * @param geometryPositionOffset New offset vector of the center of mass position to the center of geometry position
     */
    void setGeometryPositionOffset(RealVector geometryPositionOffset);

    /**
     * Function that returns the type of the physical object
     * @return Type of the physical object
     */
    PhysicalObjectType getPhysicalObjectType();

    /**
     * Function that returns the error flag
     * @return Error flag of the physical object
     */
    boolean getError();

    /**
     * Function that sets the error flag
     * @param error New Error flag of the physical object
     */
    void setError(boolean error);

    /**
     * Function that returns the collision flag
     * @return Collision flag of the physical object
     */
    boolean getCollision();

    /**
     * Function that sets the collision flag
     * @param collision New collision flag of the physical object
     */
    void setCollision(boolean collision);

    /**
     * Function that returns the id of the physical object
     * @return Id of the physical object
     */
    long getId();

    /**
     * Function that returns a list of pairs of 3D coordinates, indicating a vector on the edges of the physical object
     * @return List of pairs of 3D points, indicating a vector on the edges of the physical object
     */
    List<Map.Entry<RealVector, RealVector>> getBoundaryVectors();
    // ToDo is unnecessary with three dimensional collision detection

    /**
     * Function that computes one step of the physical behaviour of the object
     * @param deltaTms Duration of the current simulation step in milliseconds
     */
    void computePhysics(long deltaTms);

    /**
     * Function that sets the position of the center of mass and the rotation of the object, in order to place the object on the surface of the world.
     * given a x, y coordinate and a z rotation
     * @param posX X component of the position of the physical object
     * @param posY Y component of the position of the physical object
     * @param rotZ Z component of the rotation of the physical object
     */
    void putOnSurface(double posX, double posY, double rotZ);
}
