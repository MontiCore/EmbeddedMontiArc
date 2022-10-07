/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.weather;


import static org.junit.Assert.*;

import org.junit.*;

import java.time.Duration;


/**
 * Created by lukas on 02.02.17.
 */
public class WeatherTest {


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
