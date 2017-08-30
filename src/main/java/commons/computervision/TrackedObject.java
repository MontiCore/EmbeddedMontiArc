package commons.computervision;

import java.awt.*;
import java.util.LinkedList;
import java.util.List;

/**
 * Represents an detected object with its position over time
 *
 * @author Christoph Richter
 * @since 31.01.2017
 */
public class TrackedObject extends DetectedObject {
    // class fields
    // used to compute unique id
    private static int numberOfObjects = 0;

    // fields
    private int id;

    /**
     * A list of the past estimated angles to the object.
     * The newest angle is at index 0.
     */
    private List<Double> pastAngles;

    /**
     * A list of the past estimated depths of the object.
     * The newest depth is at index 0.
     */
    private List<Double> pastDepths;

    // constructor

    /**
     * Creates new Tracked objet with unique ID assigned
     */
    public TrackedObject() {
        // init lists
        pastAngles = new LinkedList<>();
        pastDepths = new LinkedList<>();

        // set id
        id = getUniqueID();
    }

    // methods

    /**
     * Adds a new detection to the track.
     *
     * @param angle  Angle of the current detection in deg
     * @param depth  Distance to the current detection
     * @param x      x coordinate of the top left point of the object bounding box
     * @param y      y coordinate of the top left point of the object bounding box
     * @param width  width of the object bounding box
     * @param height height of the object bounding box
     */
    public void addDetection(double angle, double depth, int x, int y, int width, int height) {
        pastAngles.add(0, angle);
        pastDepths.add(0, depth);
        this.setAngleDeg(angle);
        this.setDepth(depth);
        // first detection
        setImgPos(new Point(x, y));
        setBoundingBoxSize(new Point(width, height));
    }

    /**
     * Get the past angles to the object. Index 0 contains the newest angle of the object
     *
     * @return List of past depths
     */
    public List<Double> getPastAngles() {
        return pastAngles;
    }

    /**
     * Get the past depths. Index 0 contains the newest depth of the object
     *
     * @return List of past depths
     */
    public List<Double> getPastDepths() {
        return pastDepths;
    }

    /**
     * Unique id to identify the object
     *
     * @return unique id
     */
    public int getId() {
        return id;
    }

    private static int getUniqueID() {
        if (numberOfObjects == Integer.MAX_VALUE) {
            numberOfObjects = 0;
        }
        numberOfObjects++;
        return numberOfObjects;
    }
}
