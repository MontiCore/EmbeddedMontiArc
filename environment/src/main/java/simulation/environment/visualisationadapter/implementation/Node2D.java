/**
 *
 * ******************************************************************************
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
package simulation.environment.visualisationadapter.implementation;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point3D;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.SignTypeAndState;
import simulation.environment.visualisationadapter.interfaces.StreetSign;

/**
 * Created by lukas on 08.01.17.
 *
 * This class represents a Node in the Environment. It implements both EnvNode and IControllerNode.
 * The latter had to be implemented due to memory issues. Thus getId() returns the osm-id, too.
 * Note: (Normally there is a difference between getId() and getOsmId())
 */
public class Node2D implements EnvNode {

    protected Point3D point;

    protected long osmId;

    protected StreetSign sign;

    public Node2D(Number longX, Number latY) {
        this.point = new Point3D(longX.doubleValue(), latY.doubleValue(), 0.d);
    }

    public Node2D(Number x, Number y, long osmId) {
        this(x, y);
        this.osmId = osmId;
    }

    public Node2D(double x, double y, double z, long osmId) {
        this.point = new Point3D(x, y, z);
        this.osmId = osmId;
    }

    public Node2D(double x, double y, double z) {
        this.point = new Point3D(x, y, z);
    }

    @Override
    public Number getX() {
        return point.getX();
    }

    @Override
    public Number getY() {
        return point.getY();
    }

    @Override
    public Number getZ() {
        return point.getZ();
    }

    public void setZ(Number z) {
        this.point = new Point3D(point.getX(), point.getY(), z.doubleValue());
    }

    public Point3D getPoint() {
        return this.point;
    }

    @Override
    public StreetSign getStreetSign() {
        if(this.sign == null) {
            this.sign = new StreetSignImpl(SignTypeAndState.EMPTY_SIGN);
        }
        return this.sign;
    }

    public void setStreetSign(StreetSign sign) {
        this.sign = sign;
    }

    @Override
    public long getId() {
        return this.osmId;
    }


    public long getOsmId() {
        return this.osmId;
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Node2D)) return false;

        Node2D node2D = (Node2D) o;

        return point.equals(node2D.point);

    }

    @Override
    public int hashCode() {
        return point.hashCode();
    }

    public String toString() {
        return "Osm-ID: " + this.osmId + "\n\t" + this.point.toString();
    }

}