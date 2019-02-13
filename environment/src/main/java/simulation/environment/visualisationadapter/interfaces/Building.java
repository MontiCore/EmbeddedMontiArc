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
package simulation.environment.visualisationadapter.interfaces;

/**
 * Created by lukas on 15.12.16.
 *
 * An interface for buildings in the environment
 * Currently unused
 *
 * Update by Florisa on 31.01.19.
 */
public interface Building extends EnvObject{
    /**
     * lists type of buildings
     */
    public enum BuildingTypes{UNIVERSITY, HOUSE, GARAGES, CHURCH}

    /**
     * @return the type of buildings
     */

    public abstract BuildingTypes getBuildingTypes();
}