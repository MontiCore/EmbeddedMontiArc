/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.util;

import org.junit.Rule;
import org.junit.Test;
import org.junit.contrib.java.lang.system.EnvironmentVariables;
import org.junit.runner.RunWith;
import org.powermock.api.mockito.PowerMockito;
import org.powermock.core.classloader.annotations.PowerMockIgnore;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;
import de.rwth.montisim.simulation.environment.World;
import de.rwth.montisim.simulation.environment.osm.ParserSettings;
import de.rwth.montisim.simulation.environment.weather.WeatherSettings;
import de.rwth.montisim.simulation.util.ServerRequest;

import static org.junit.Assert.assertEquals;

@RunWith(PowerMockRunner.class)
@PrepareForTest(ServerRequest.class)
@PowerMockIgnore("javax.net.ssl.*")
public class ChargingStationNavigatorTest {
    @Rule
    public final EnvironmentVariables environmentVariables = new EnvironmentVariables();


    @Test
    public void getNearestChargingStationFromLocalSector() throws Exception {
        World.init(
                new ParserSettings(
                        getClass().getResourceAsStream("/map_charging_station.osm"),
                        ParserSettings.ZCoordinates.ALLZERO, "supercharger"),
                new WeatherSettings());
        long id = ChargingStationNavigator.getNearestChargingStationFromLocalSector(267028542L);
        assertEquals(4846216969L, id);
    }

    @Test
    public void getNearestChargingStationFromServer() throws Exception {
        World.init(
                new ParserSettings(
                        getClass().getResourceAsStream("/map_charging_station.osm"),
                        ParserSettings.ZCoordinates.ALLZERO, "supercharger"),
                new WeatherSettings());

        environmentVariables.set("SIM_SERVER", "");
        environmentVariables.set("SIM_PORT", "");
        // Should return 0L since server is not configured
        long id = ChargingStationNavigator.getNearestChargingStationFromServer("", 267028542L);
        assertEquals(0L, id);

        environmentVariables.set("SIM_SERVER", "localhost");
        environmentVariables.set("SIM_PORT", "8888");
        PowerMockito.mockStatic(ServerRequest.class);
        PowerMockito.when(ServerRequest.sendChargingStationRequest(
                "localhost", "8888", "id", 1111111)).thenReturn("999999");
        // Should query from server since server is provided
        id = ChargingStationNavigator.getNearestChargingStationFromServer("id", 1111111);
        assertEquals(999999L, id);

        PowerMockito.when(ServerRequest.sendChargingStationRequest(
                "localhost", "8888", "id", 1111111)).thenReturn("randomstuff");
        // Should query from server since server is provided
        id = ChargingStationNavigator.getNearestChargingStationFromServer("id", 1111111);
        assertEquals(0, id);
    }
}