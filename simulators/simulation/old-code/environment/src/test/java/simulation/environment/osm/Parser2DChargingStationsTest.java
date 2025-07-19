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

public class Parser2DChargingStationsTest {
    @Test
    public void testChargingStations() throws Exception {
        InputStream in = getClass().getResourceAsStream("/map_charging_station2.osm");
        IParser parser = new Parser2D(new ParserSettings(in, ParserSettings.ZCoordinates.ALLZERO));
        parser.parse();
        System.out.println("Name, capacity, osmID and position for Charging Station Objects: " + parser.getChargingStations());
        System.out.println(" ");
        int number = parser.getChargingStations().size();
        System.out.println("Number of Charging Station Objects: " + number);

        assertEquals(4, number);
    }
}
