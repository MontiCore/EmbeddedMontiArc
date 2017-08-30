package commons.computervision;

import java.awt.image.BufferedImage;
import java.util.LinkedList;

/**
 * Interface to detect objects in a image
 *
 * @author Christoph Richter
 * @since 24.01.2017
 */
public interface DetectObject {
    public LinkedList<DetectedObject> detectObjects(BufferedImage image);
}
