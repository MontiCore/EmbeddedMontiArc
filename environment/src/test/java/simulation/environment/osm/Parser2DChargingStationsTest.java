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

import static org.junit.Assert.*;
import org.junit.*;
import java.io.InputStream;

public class Parser2DChargingStationsTest {
    @Test
    public void testChargingStations() throws Exception {
        InputStream in = getClass().getResourceAsStream("/map_charging_station2.osm");
        IParser parser = new Parser2D(new ParserSettings(in, ParserSettings.ZCoordinates.ALLZERO));
        parser.parse();
        System.out.println("Name, capacity, osmID and position for Charging Station Objects: "+ parser.getChargingStations());
        System.out.println(" ");
        int number = parser.getChargingStations().size();
        System.out.println("Number of Charging Station Objects: " + number);

        assertEquals(4, number);
    }
}
