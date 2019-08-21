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
package simulation.environment.osm;

import simulation.environment.object.ChargingStation;
import simulation.environment.visualisationadapter.interfaces.Building;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
import simulation.environment.visualisationadapter.interfaces.VisualisationEnvironmentContainer;
import simulation.environment.visualisationadapter.interfaces.Waterway;

import java.util.Collection;

/**
 * Created by lukas on 08.01.17.
 * An Interface which specifies the methods of an OSM-Parser
 * to remove the restriction to OSM remove getDataSet() or switch from InMemoryMapDataSet to an appropriate Interface
 */
public interface IParser {
    /**
     * parses OSM-Data
     * @throws Exception
     */
    public abstract void parse() throws Exception;


    /**
     * @return a Collection of all Streets parsed
     */
    public abstract Collection<EnvStreet> getStreets();

    /**
     * @return a collection of all Buildings parsed
     */
    public abstract Collection<Building> getBuildings();

    /**
     * @return a collection of all Buildings parsed
     */
    public abstract Collection<Waterway> getWaterways();

    /**
     * @return a collection of all Charging Stations parsed
     */
    public abstract Collection<ChargingStation> getChargingStations();


    /**
     * @return an EnvironmentContainer
     */
    public abstract VisualisationEnvironmentContainer getContainer();
}