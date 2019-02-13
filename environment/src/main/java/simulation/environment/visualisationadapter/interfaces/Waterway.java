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

import java.util.Collection;

public interface Waterway extends EnvObject {
    //TODO: This should be moved to commons, controller needs to be able to access these constant values
    /**
     * Lists Type of Streets
     */
    public enum WaterTypes {RIVER, STREAM, DITCH};

    /**
     * set STREET_WIDTH to 6 meters
     */
    public final double RIVER_WIDTH = 9;



    /**
     *
     * @return the width of this Waterway
     */
    public abstract Number getWaterwayWidth();


    /**
     *
     * @return The Type of the Street
     */
    public abstract WaterTypes getWaterType();
}
