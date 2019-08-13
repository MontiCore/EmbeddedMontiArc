/**
 *
 *  ******************************************************************************
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
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map;

/**
 * Created by lukas on 31.01.17.
 */
public class Adjacency implements IAdjacency{

    private IControllerNode node1;
    private IControllerNode node2;

    private long streetId;

    private double distance;


    public Adjacency(IControllerNode node1, IControllerNode node2) {
        this.node1 = node1;
        this.node2 = node2;
        this.distance = node1.getPoint().distance(node2.getPoint());
    }

    @Override
    public IControllerNode getNode1() {
        return this.node1;
    }

    @Override
    public IControllerNode getNode2() {
        return this.node2;
    }

    @Override
    public double getDistance() {
        return this.distance;
    }
}
