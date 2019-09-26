package simulation.vehicle;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map.IControllerNode;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point3D;

public class ControllerNodeImpl implements IControllerNode {
    private Point3D position;
    private long osmId;

    public ControllerNodeImpl(Point3D position, long osmId) {
        this.position = position;
        this.osmId = osmId;
    }

    @Override
    public Point3D getPoint() {
        return position;
    }

    @Override
    public long getId() {
        return osmId;
    }

    @Override
    public long getOsmId() {
        return osmId;
    }
}
