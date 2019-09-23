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
package simulation.environment.util;

import org.junit.Rule;
import org.junit.Test;
import org.junit.contrib.java.lang.system.EnvironmentVariables;
import org.junit.runner.RunWith;
import org.powermock.api.mockito.PowerMockito;
import org.powermock.core.classloader.annotations.PowerMockIgnore;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;
import simulation.environment.WorldModel;
import simulation.environment.osm.ParserSettings;
import simulation.environment.weather.WeatherSettings;
import simulation.util.ServerRequest;

import static org.junit.Assert.assertEquals;

@RunWith(PowerMockRunner.class)
@PrepareForTest(ServerRequest.class)
@PowerMockIgnore("javax.net.ssl.*")
public class ChargingStationNavigatorTest {
    @Rule
    public final EnvironmentVariables environmentVariables = new EnvironmentVariables();


    @Test
    public void getNearestChargingStationFromLocalSector() throws Exception {
        WorldModel.init(
                new ParserSettings(
                        getClass().getResourceAsStream("/map_charging_station.osm"),
                        ParserSettings.ZCoordinates.ALLZERO, "supercharger"),
                new WeatherSettings());
        long id = ChargingStationNavigator.getNearestChargingStationFromLocalSector(267028542L);
        assertEquals(4846216969L, id);
    }

    @Test
    public void getNearestChargingStationFromServer() throws Exception {
        WorldModel.init(
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