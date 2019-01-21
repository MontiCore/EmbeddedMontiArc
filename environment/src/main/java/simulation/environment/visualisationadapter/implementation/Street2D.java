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

import simulation.environment.visualisationadapter.interfaces.EnvIntersection;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
import simulation.environment.visualisationadapter.interfaces.EnvTag;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Created by lukas on 08.01.17.
 *
 * This Class represents streets
 */
public class Street2D extends EnvObject2D implements EnvStreet {

    private boolean isOneWay;
    private Number speedLimit;
    private ArrayList<EnvNode> intersections;
    private StreetTypes streetType;
    private StreetPavements streetPavements;



    public Street2D(List<EnvNode> nodes, Number speedLimit, Collection<EnvIntersection> intersections, boolean isOneWay) {
        super(nodes, EnvTag.STREET);
        init(speedLimit, intersections, isOneWay);
    }

    public Street2D(List<EnvNode> nodes, Number speedLimit, Collection<EnvIntersection> intersections, long osmId, boolean isOneWay) {
        super(nodes, EnvTag.STREET, osmId);
        init(speedLimit, intersections, isOneWay);
    }


    public Street2D(List<EnvNode> nodes, Number speedLimit, Collection<EnvIntersection> intersections, long osmId, boolean isOneWay, StreetTypes streetType) {
        super(nodes, EnvTag.STREET, osmId);
        init(speedLimit, intersections, isOneWay, streetType);
    }


    public Street2D(List<EnvNode> nodes, Number speedLimit, Collection<EnvIntersection> intersections, long osmId, boolean isOneWay, StreetTypes streetType, StreetPavements streetPavements) {
        super(nodes, EnvTag.STREET, osmId);
        init(speedLimit, intersections, isOneWay, streetType, streetPavements);
    }

    private void init(Number speedLimit, Collection<EnvIntersection> intersections, boolean isOneWay) {
        this.speedLimit = speedLimit;
        this.isOneWay = isOneWay;
        if (intersections == null) {
            this.intersections = new ArrayList<>();
        } else {
            this.intersections = new ArrayList<>(intersections);
        }
    }


    private void init(Number speedLimit, Collection<EnvIntersection> intersections, boolean isOneWay, StreetTypes streetType) {
        this.speedLimit = speedLimit;
        this.isOneWay = isOneWay;
        this.streetType = streetType;
        if (intersections == null) {
            this.intersections = new ArrayList<>();
        } else {
            this.intersections = new ArrayList<>(intersections);
        }
    }

    private void init(Number speedLimit, Collection<EnvIntersection> intersections, boolean isOneWay, StreetTypes streetType, StreetPavements streetPavements) {
        this.speedLimit = speedLimit;
        this.isOneWay = isOneWay;
        this.streetType = streetType;
        this.streetPavements = streetPavements;
        if (intersections == null) {
            this.intersections = new ArrayList<>();
        } else {
            this.intersections = new ArrayList<>(intersections);
        }
    }

    public String toString() {
        return this.nodes.toString();
    }

    @Override
    public Number getSpeedLimit() {
        return speedLimit;
    }

    @Override
    public Collection<EnvNode> getIntersections() {
        return intersections;
    }

    @Override
    public Number getStreetWidth() {
        return EnvStreet.STREET_WIDTH;
    }

    @Override
    public boolean isOneWay() {
        return isOneWay;
    }

    @Override
    public StreetTypes getStreetType(){ return streetType; }

    @Override
    public StreetPavements getStreetPavement() { return streetPavements; }

}