/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.visualisationadapter;

import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.osm.Coordinates;

/**
 * This class represents a Node in the Environment. It implements both EnvNode and IControllerNode.
 * The latter had to be implemented due to memory issues. Thus getId() returns the osm-id, too.
 * Note: (Normally there is a difference between getId() and getOsmId())
 */
public class EnvNode {
    public Vec3 point;
    public long osmId;
    protected StreetSign sign;

    public EnvNode(Coordinates coords) {
        this.point = new Vec3(coords, 0.d);
    }

    public EnvNode(Coordinates coords, long osmId) {
        this(coords);
        this.osmId = osmId;
    }

    public EnvNode(Vec3 pos, long osmId) {
        this.point = pos;
        this.osmId = osmId;
    }

    public EnvNode(Vec3 pos) {
        this.point = pos;
    }

    public void setZ(Number z) {
        this.point.z = z.doubleValue();
    }


    public StreetSign getStreetSign() {
        if (this.sign == null) {
            this.sign = new StreetSign(SignTypeAndState.EMPTY_SIGN);
        }
        return this.sign;
    }

    public void setStreetSign(StreetSign sign) {
        this.sign = sign;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof EnvNode)) return false;

        EnvNode node2D = (EnvNode) o;

        return point.equals(node2D.point);

    }

    @Override
    public int hashCode() {
        return point.hashCode();
    }

    public String toString() {
        return "{osmid: " + this.osmId + ", pos:" + this.point.toString() + "}";
    }

}
