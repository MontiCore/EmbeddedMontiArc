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
package simulation.environment;

import junit.framework.TestCase;
import org.junit.Test;
import simulation.environment.osm.ParserSettings;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
import simulation.environment.visualisationadapter.interfaces.SignTypeAndState;
import simulation.environment.weather.WeatherSettings;

public class WorldModelTest extends TestCase {
    @Test
    public void testApp() throws Exception {
        try {
            World world = WorldModel.getInstance();
        } catch (Exception e) {
            fail();
        }


        WorldModel.init(new ParserSettings(getClass().getResourceAsStream("/map_ahornstrasse.osm"),
                                            ParserSettings.ZCoordinates.STATIC), new WeatherSettings());

        World world = WorldModel.getInstance();

        boolean allZero = true;

        for(EnvStreet s : world.getContainer().getStreets()) {
            for(EnvNode n : s.getNodes()) {
                allZero &= n.getZ().doubleValue() == 0;
            }
        }

        assertFalse(allZero);


        WorldModel.init(new ParserSettings(getClass().getResourceAsStream("/map_ahornstrasse.osm"),
                ParserSettings.ZCoordinates.ALLZERO), new WeatherSettings());

        world = WorldModel.getInstance();

        allZero = true;

        for(EnvStreet s : world.getContainer().getStreets()) {
            for(EnvNode n : s.getNodes()) {
                allZero &= n.getZ().doubleValue() == 0;
            }
        }

        assertTrue(allZero);


        WorldModel.init(new ParserSettings(getClass().getResourceAsStream("/map_ahornstrasse.osm"),
                ParserSettings.ZCoordinates.RANDOM), new WeatherSettings());

        world = WorldModel.getInstance();

        allZero = true;

        for(EnvStreet s : world.getContainer().getStreets()) {
            for(EnvNode n : s.getNodes()) {
                allZero &= n.getZ().doubleValue() == 0;
            }
        }

        assertFalse(allZero);


        WorldModel.init(new ParserSettings(getClass().getResourceAsStream("/map_ahornstrasse.osm"),
                ParserSettings.ZCoordinates.ALLZERO), new WeatherSettings());

        world = WorldModel.getInstance();

        assertTrue(world.getContainer().getBounds().getMinX() < 0);
        assertTrue(world.getContainer().getBounds().getMinY() < 0);
        assertTrue(world.getContainer().getBounds().getMinZ() == 0);

        WorldModel.init(new ParserSettings(getClass().getResourceAsStream("/map_ahornstrasse.osm"),
                ParserSettings.ZCoordinates.STATIC), new WeatherSettings());

        world = WorldModel.getInstance();
        assertTrue(world.getContainer().getBounds().getMinX() < 0);
        assertTrue(world.getContainer().getBounds().getMinY() < 0);
        assertTrue(world.getContainer().getBounds().getMinZ() >= 0);

        WorldModel.init(new ParserSettings(getClass().getResourceAsStream("/map_ahornstrasse.osm"),
                ParserSettings.ZCoordinates.RANDOM), new WeatherSettings());

        world = WorldModel.getInstance();
        assertTrue(world.getContainer().getBounds().getMinX() < 0);
        assertTrue(world.getContainer().getBounds().getMinY() < 0);
        assertTrue(world.getContainer().getBounds().getMinZ() >= 0);


        for(EnvStreet s : world.getContainer().getStreets()) {
            for(EnvNode n : s.getNodes()) {
                if(n.getStreetSign().getType() != SignTypeAndState.EMPTY_SIGN) {
                    System.out.println(n.getStreetSign().getX1());
                    System.out.println(n.getStreetSign().getX2());
                    assertTrue(n.getStreetSign().isOne() || n.getStreetSign().isTwo());
                }
            }
        }

        WorldModel.init(new ParserSettings("/map_ahornstrasse.osm", ParserSettings.ZCoordinates.FROM_FILE),
                new WeatherSettings());
        world = WorldModel.getInstance();

        // Test height of some random nodes
        // Reference values taken from https://www.freemaptools.com/elevation-finder.htm
        for(EnvStreet s : world.getContainer().getStreets()) {
            for(EnvNode n : s.getNodes()) {
                if (n.getOsmId() == 1830204382L) {
                    assertEquals(237, Math.round(n.getZ().doubleValue()));
                }
                else if (n.getOsmId() == 4180733590L) {
                    assertEquals(222, Math.round(n.getZ().doubleValue()));
                }
                else if (n.getOsmId() == 206176292L) {
                    assertEquals(210, Math.round(n.getZ().doubleValue()));
                }
                else if (n.getOsmId() == 36831057L) {
                    assertEquals(209, Math.round(n.getZ().doubleValue()));
                }
                else if (n.getOsmId() == 60533928) {
                    assertEquals(207, Math.round(n.getZ().doubleValue()));
                }
                else if (n.getOsmId() == 450648425) {
                    assertEquals(228, Math.round(n.getZ().doubleValue()));
                }
            }
        }

        WorldModel.init(new ParserSettings("/map_ahornstrasse.osm", ParserSettings.ZCoordinates.FROM_FILE),
                new WeatherSettings());
        world = WorldModel.getInstance();
        assertNotNull(world.getContainer().getHeightMap());
        assertNotNull(world.getContainer().getHeightMapMinPoint());
        assertNotNull(world.getContainer().getHeightMapMaxPoint());
        assertNotSame(0.0, world.getContainer().getHeightMapDeltaX());
        assertNotSame(0.0, world.getContainer().getHeightMapDeltaY());
    }
}