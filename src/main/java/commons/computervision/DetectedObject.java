package commons.computervision;

import commons.simulation.PhysicalObjectType;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import java.awt.*;

/**
 * Represents a detected object by angle to the object and the estimated depth. If depth estimation is not available
 * depth will be -1! Is used to further process the object detection results.
 *
 * @author Christoph Richter
 * @since 24.01.2017
 */
public class DetectedObject {
    // fields
    /**
     * Angle to the detected object (deg)
     */
    private double angle = 0;
    /**
     * Estimated depth to the object
     */
    private double depth = -1;
    /**
     * Upper left position in the image
     */
    private Point imgPos;
    /**
     * Bounding box size from the screen position (x = width; y = height)
     */

    /**
     *Type of the detected physical Object
     */
    private PhysicalObjectType type;

    private Point boundingBoxSize;
    // constructors

    /**
     * Empty constructor
     */
    public DetectedObject() {
    }

    /**
     * Constructor using params:
     * Constructor for Computervision
     *
     * @param angle angle to the detected object (deg)
     * @param depth estimated depth to the object
     */
    public DetectedObject(double angle, double depth) {
        this.angle = angle;
        this.depth = depth;
        this.type = PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR_DEFAULT;
    }

    /**
     * Constructor for ObjectSensor
     * @param angle
     * @param depth
     * @param type
     */
    public DetectedObject(double angle, double depth, PhysicalObjectType type) {
        this.angle = angle;
        this.depth = depth;
        this.type = type;
    }

    // methods

    public double getAngleDeg() {
        return angle;
    }

    public void setAngleDeg(double angle) {
        this.angle = angle;
    }

    public double getAngleRad() {
        return Math.toRadians(angle);
    }

    public void setAngleRad(double angle) {
        this.angle = Math.toDegrees(angle);
    }

    public double getDepth() {
        return depth;
    }

    public void setDepth(double depth) {
        this.depth = depth;
    }

    public void setPhysicalObjectType(PhysicalObjectType type){this.type=type;}

    public PhysicalObjectType getPhysicalObjectType() {return type;}

    /**
     * Calculates the vector to the object. If depth is negative the null vector will be returned!
     *
     * @return the vector to the detected object from the current camera position
     */
    public RealVector getVectorToObject() {
        RealVector result = new ArrayRealVector(2, 0);
        if (depth > 0) {
            // get direction
            double x = Math.cos(angle);
            double y = Math.sin(angle);
            result.setEntry(0, x);
            result.setEntry(1, y);
            // scale vector
            double length = depth / Math.cos(angle);
            result = result.mapMultiply(length);
        }
        // return
        return result;
    }

    public Point getImgPos() {
        return imgPos;
    }

    public void setImgPos(Point imgPos) {
        this.imgPos = imgPos;
    }

    public Point getBoundingBoxSize() {
        return boundingBoxSize;
    }

    public void setBoundingBoxSize(Point boundingBoxSize) {
        this.boundingBoxSize = boundingBoxSize;
    }
}
