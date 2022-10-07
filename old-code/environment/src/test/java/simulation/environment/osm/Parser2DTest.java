/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.osm;

import static org.junit.Assert.*;

import org.junit.*;

import java.io.InputStream;

/**
 * Created by lukas on 08.01.17.
 */
public class Parser2DTest {


    @Test
    public void testApp() throws Exception {
        // test will always failed with maven because it should read the file from inside the jar.
        // String filePath = "src/test/data/map_buildings_test.osm";
        InputStream in = getClass().getResourceAsStream("/map.osm");
        IParser parser = new Parser2D(new ParserSettings(in, ParserSettings.ZCoordinates.ALLZERO));
        parser.parse();

        //  System.out.println(parser.getWaterways());
        System.out.println("LAT and LON Coordinates for Waterway Objects: " + parser.getWaterways());
        System.out.println(" ");
        System.out.println(" ");
        System.out.println("Number of Waterway Objects: " + parser.getWaterways().size());

        System.out.println(" ");
        System.out.println(" ");

        System.out.println("LAT and LON Coordinates for Building Objects: " + parser.getBuildings());
        System.out.println(" ");
        System.out.println(" ");
        System.out.println("Number of Building Objects: " + parser.getBuildings().size());

        System.out.println(" ");
        System.out.println(" ");

        System.out.println("LAT and LON Coordinates for Waterway Objects: " + parser.getStreets());
        System.out.println(" ");
        System.out.println(" ");
        System.out.println("Number of Street Objects: " + parser.getStreets().size());
        assertNotNull(parser.getStreets());


    }
}
