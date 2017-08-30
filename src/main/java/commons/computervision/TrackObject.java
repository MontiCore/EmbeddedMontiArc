package commons.computervision;

import java.awt.image.BufferedImage;
import java.util.LinkedList;

/**
 * Interface to track object positions over time.
 *
 * @author Christoph Richter
 * @since 31.01.2017
 */
public interface TrackObject {
    public LinkedList<TrackedObject> trackObjects(BufferedImage image);

    public LinkedList<TrackedObject> trackObjects(BufferedImage image, LinkedList<DetectedObject> detectedObjects);
}
