package commons.simulation;


import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.List;
import java.util.Map;

/**
 * Interface that represents a physical object in a 3D environment
 */
public interface PhysicalObject {

    /**
     * Function that returns the type of the object
     * @return Type of the object
     */
    PhysicalObjectType getPhysicalObjectType();

    /**
     * Function that returns a vector with the x, y and z coordinates of the object
     * This refers to the center position of the geometry object (i.e. NOT mass point position)
     * @return Vector with x, y, z coordinates of the object center
     */
    RealVector getGeometryPos();

    /**
     * Function that returns a matrix with the rotation of the object
     * @return Matrix with the rotation of the object
     */
    RealMatrix getGeometryRot();

    /**
     * Function that returns the width of an object. Only meaningful for a box object, otherwise returns 0.0.
     * @return Width of a box object, otherwise 0.0
     */
    double getWidth();

    /**
     * Function that returns the length of an object. Only meaningful for a box object, otherwise returns 0.0.
     * @return Length of a box object, otherwise 0.0
     */
    double getLength();

    /**
     * Function that returns the height of an object. Only meaningful for a box object, otherwise returns 0.0.
     * @return Height of a box object, otherwise 0.0
     */
    double getHeight();

    /**
     * Function that returns the z offset of an object. This is used to represent the wheel radius for vehicles, otherwise 0.0
     * @return Z offset of an object, i.e. wheel radius for vehicles, otherwise 0.0
     */
    double getOffsetZ();

    /**
     * Function that returns a boolean indicating if an object had a collision
     * @return Boolean that indicates a collision of that object
     */
    boolean getCollision();

    /**
     * Function that sets collision for this object
     * @param collision Boolean that indicates a collision of that object
     */
    void setCollision(boolean collision);

    /**
     * Returns the unique ID of the object. Valid IDs are positive numbers.
     * @return Unique ID
     */
    long getId();

    /**
     * Returns the acceleration of the object
     * @return Acceleration in m/s^2
     */
    RealVector getAcceleration();

    /**
     * Returns the velocity of the object
     * @return Velocity in m/s
     */
    RealVector getVelocity();

    /**
     * Returns the steering angle of the object. The steering angle is given in radians. 0 means the car is
     * driving straight forward, negative values mean the car is steering left and positive values mean the
     * car is steering right.
     * @return Steering angle in radians.
     */
    double getSteeringAngle();

    /**
     * Function that returns a list of pairs of 3D points, indicating the beginning and end of a vector in absolute 3D global coordinates
     * These vectors are checked for overlaps / going under the map in collision detection
     * @return List of pairs of 3D points, indicating the beginning and end of a vector in absolute 3D global coordinates
     */
    List<Map.Entry<RealVector, RealVector>> getBoundaryVectors();

    /**
     * Returns The Position of the FrontLeftWheel Position for DistanceCalculating. For Objects which are not
     * PhysicalVehicle's it returns a null Vector.
     * @return Vector of the Position.
     */
    RealVector getFrontLeftWheelGeometryPos();

    /**
     * Returns The Position of the FrontRightWheel Position for DistanceCalculating. For Objects which are not
     * PhysicalVehicle's it returns a null Vector.
     * @return Vector of the Position.
     */
    RealVector getFrontRightWheelGeometryPos();

    /**
     * Returns The Position of the BackLeftWheel Position for DistanceCalculating. For Objects which are not
     * PhysicalVehicle's it returns a null Vector.
     * @return Vector of the Position.
     */
    RealVector getBackLeftWheelGeometryPos();

    /**
     * Returns The Position of the BackRightWheel Position for DistanceCalculating. For Objects which are not
     * PhysicalVehicle's it returns a null Vector.
     * @return Vector of the Position.
     */
    RealVector getBackRightWheelGeometryPos();
}
