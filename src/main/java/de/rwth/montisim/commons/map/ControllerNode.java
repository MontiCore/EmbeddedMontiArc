/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.map;

import de.rwth.montisim.commons.utils.Vec3;
import java.util.HashMap;

public class ControllerNode {
    public static final long INTERPOLATED_NODE = -1;

    public static final double INTERPOLATION_DISTANCE = 2;

    private static int counter = 0;

    public Vec3 pos;

    public long id;
    public long osmId;

    private static HashMap<Vec3, Long> ids = new HashMap<>();

    /// Takes ownership of the position.
    public ControllerNode(Vec3 p, long osmId) {
        this.pos = p;
        if (!ids.containsKey(p)) {
            this.id = ++counter;
            ids.put(p, this.id);
        } else {
            this.id = ids.get(p);
        }
        this.osmId = osmId;
    }

    public String toString() {
        return "{id: " + id + ", osmid: " + osmId + ", pos: " + pos.toString() + "}";
    }

    public static int getCounter() {
        return counter;
    }
}
