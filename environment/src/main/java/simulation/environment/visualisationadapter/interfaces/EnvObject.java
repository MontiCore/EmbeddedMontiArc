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

import java.util.List;

/**
 * Created by Danilo 07.02.2019.
 *
 * An interface for all Objects in the environment
 */
public interface EnvObject {
    /**
     * @return a List of the Nodes this object consists of
     */
    public abstract List<EnvNode> getNodes();

    /**
     * @return a Tag containing more information on this Object
     */
    public abstract EnvTag getTag();

    /**
     * @return the OpenStreetMap of this Object
     */
    public abstract long getOsmId();
}