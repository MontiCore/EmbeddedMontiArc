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
import simulation.environment.visualisationadapter.interfaces.EnvTag;
import simulation.environment.visualisationadapter.interfaces.Waterway;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class Waterway2D extends EnvObject2D implements Waterway {



    public Waterway.WaterTypes waterType;

    public Waterway2D(List<EnvNode> nodes) {
        super(nodes, EnvTag.WATERWAY);
    }

    public Waterway2D(List<EnvNode> nodes,  long osmId) {
        super(nodes, EnvTag.WATERWAY, osmId);

    }

    public Waterway2D(List<EnvNode> nodes,  long osmId, Waterway.WaterTypes waterType) {
        super(nodes, EnvTag.WATERWAY, osmId);

    }





    public String toString() {
        return this.nodes.toString();
    }





    @Override
    public Number getWaterwayWidth() {
        return Waterway.RIVER_WIDTH;
    }



    @Override
    public Waterway.WaterTypes getWaterType(){ return waterType; }
}
