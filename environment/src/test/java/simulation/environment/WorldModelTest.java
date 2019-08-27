/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.environment;

import org.junit.*;
import static  org.junit.Assert.*;
import simulation.environment.osm.IParser;
import simulation.environment.osm.Parser2D;
import simulation.environment.osm.ParserSettings;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
import simulation.environment.visualisationadapter.interfaces.SignTypeAndState;
import simulation.environment.weather.WeatherSettings;

import java.io.InputStream;

public class WorldModelTest{

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


        WorldModel.init(new ParserSettings(getClass().getResourceAsStream("/main_resources_aachen.osm"),
                ParserSettings.ZCoordinates.ALLZERO), new WeatherSettings());

        world = WorldModel.getInstance();

        System.out.println("Map Building test");
        System.out.println(" Building Nodes");

        System.out.println(world.getContainer().getStreets().size());
        System.out.println(world.getContainer().getBuildings().size());
        System.out.println(world.getContainer().getWaterway().size());

        //System.out.println(" Street Nodes");

       // System.out.println(world.getContainer().getStreets());


        assertTrue(world.getContainer().getBuildings().size() > 0);
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
