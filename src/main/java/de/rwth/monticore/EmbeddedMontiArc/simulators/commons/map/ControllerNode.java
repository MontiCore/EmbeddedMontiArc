/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point3D;
import java.util.HashMap;

/**
 * Created by lukas on 31.01.17.
 */
public class ControllerNode implements IControllerNode{

    private static int counter = 0;

    private Point3D p;

    private long id;
    private long osmId;

    private static HashMap<Point3D, Long> ids = new HashMap<>();

    public ControllerNode(Point3D p, long osmId) {
        this.p = p;
        if(!ids.containsKey(p)) {
            this.id = ++counter;
            ids.put(p, this.id);
        } else {
            this.id = ids.get(p);
        }
        this.osmId = osmId;
    }

    @Override
    public Point3D getPoint() {
        return this.p;
    }

    @Override
    public long getId() {
        return this.id;
    }

    @Override
    public long getOsmId() {
        return this.osmId;
    }

    public String toString() {
        return p.toString();
    }

    public static int getCounter() {
        return counter;
    }
}
