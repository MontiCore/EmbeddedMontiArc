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

import java.io.InputStream;

/**
 * Created by lukas on 16.02.17.
 *
 * A container that contains the input stream and the strategy to generate z-Coordinates
 */
public class ParserSettings {
    public enum ZCoordinates {
        ALLZERO, RANDOM, STATIC, FROM_FILE
    }

    public InputStream in;
    public ZCoordinates z;

    public ParserSettings(String in, ZCoordinates z) {
        this.in = getClass().getResourceAsStream(in);
        this.z = z;
    }

    @Deprecated
    public ParserSettings(InputStream in, ZCoordinates z) {
        this.in = in;
        this.z = z;
    }
}