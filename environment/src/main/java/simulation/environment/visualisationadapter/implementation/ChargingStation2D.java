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

import javafx.geometry.Point3D;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvTag;
import simulation.environment.visualisationadapter.interfaces.EnvChargingStation;

import java.util.List;

public class ChargingStation2D extends EnvObject2D implements EnvChargingStation {

    private int capacity;
    private String name;
    private RealVector center;

    public ChargingStation2D(List<EnvNode> nodes) {
        super(nodes, EnvTag.CHARGING_STATION);
        this.center = computeCenter();
    }

    public ChargingStation2D(List<EnvNode> nodes, long osmId) {
        super(nodes, EnvTag.CHARGING_STATION, osmId);
        this.center = computeCenter();
    }

    public ChargingStation2D(List<EnvNode> nodes, long osmId, int capacity) {
        super(nodes, EnvTag.CHARGING_STATION, osmId);
        this.capacity = capacity;
        this.center = computeCenter();
    }

    public ChargingStation2D(List<EnvNode> nodes, long osmId, int capacity, String name) {
        super(nodes, EnvTag.CHARGING_STATION, osmId);
        this.capacity = capacity;
        this.name = name;
    }

    public RealVector getCenter(){
        return center;
    }

    private RealVector computeCenter(){
        double x = 0;
        double y = 0;
        double z = 0;

        for (EnvNode node: this.nodes){
            x += node.getPoint().getX();
            y += node.getPoint().getY();
            z += node.getPoint().getZ();
        }

        int numNodes = nodes.size();
        return new ArrayRealVector(new double[]{x/numNodes, y/numNodes, z/numNodes});
    }

    @Override
    public int getCapacity() {
        return capacity;
    }

    @Override
    public String getName() {
        return name;
    }
}
