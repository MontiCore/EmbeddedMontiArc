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
package simulation.environment.weather;


import static  org.junit.Assert.*;
import org.junit.*;

import java.time.Duration;



/**
 * Created by lukas on 02.02.17.
 */
public class WeatherTest  {


	@Test
    public void testApp() throws Exception {
        Weather w = new Weather(new WeatherSettings(10l));

        double oldWeather = w.getWeather();
        w.executeLoopIteration(Duration.ofMillis(10l));
        assertFalse((oldWeather - w.getWeather() == 0));

        w = new Weather(new WeatherSettings(0.2));
        assertFalse(w.isRain());
        w.executeLoopIteration(Duration.ofMillis(Long.MAX_VALUE));
        assertFalse(w.isRain());
        assertEquals(0.2, w.getWeather(), 0.0001);

        w = new Weather(new WeatherSettings());
        assertTrue(w.getNextWeatherChange() > 0);
    }
}