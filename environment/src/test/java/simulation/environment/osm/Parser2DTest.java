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

import junit.framework.Test;
import junit.framework.TestCase;
import junit.framework.TestSuite;
import java.io.InputStream;

/**
 * Created by lukas on 08.01.17.
 */
public class Parser2DTest extends TestCase {
    /**
     * Create the test case
     *
     * @param testName name of the test case
     */
    public Parser2DTest(String testName) {
        super(testName);
    }

    /**
     * @return the suite of tests being tested
     */
    public static Test suite() {
        return new TestSuite(Parser2DTest.class);
    }


    public void testApp() throws Exception {
        // test will always failed with maven because it should read the file from inside the jar.
        // String filePath = "src/test/data/map_buildings_test.osm";
        InputStream in = getClass().getResourceAsStream("/map.osm");
        IParser parser = new Parser2D(new ParserSettings(in, ParserSettings.ZCoordinates.ALLZERO));
        parser.parse();

      //  System.out.println(parser.getWaterways());
        System.out.println("LAT and LON Coordinates for Waterway Objects: "+parser.getWaterways());
        System.out.println(" ");
        System.out.println(" ");
        System.out.println("Number of Waterway Objects: " +parser.getWaterways().size());

        System.out.println(" ");
        System.out.println(" ");

        System.out.println("LAT and LON Coordinates for Building Objects: "+parser.getBuildings());
        System.out.println(" ");
        System.out.println(" ");
        System.out.println("Number of Building Objects: " +parser.getBuildings().size());

        System.out.println(" ");
        System.out.println(" ");

        System.out.println("LAT and LON Coordinates for Waterway Objects: "+parser.getStreets());
        System.out.println(" ");
        System.out.println(" ");
        System.out.println("Number of Street Objects: " +parser.getStreets().size());
        assertNotNull(parser.getStreets());


    }
}