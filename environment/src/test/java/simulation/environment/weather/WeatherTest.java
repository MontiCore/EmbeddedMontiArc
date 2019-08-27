/* (c) https://github.com/MontiCore/monticore */
package simulation.environment.weather;

import org.junit.*;
import static  org.junit.Assert.*;

/**
 * Created by lukas on 02.02.17.
 */
public class WeatherTest  {


	@Test
    public void testApp() throws Exception {
        Weather w = new Weather(new WeatherSettings(10l));

        double oldWeather = w.getWeather();
        w.executeLoopIteration(10l);
        assertFalse((oldWeather - w.getWeather() == 0));

        w = new Weather(new WeatherSettings(0.2));
        assertFalse(w.isRain());
        w.executeLoopIteration(Long.MAX_VALUE);
        assertFalse(w.isRain());
        assertEquals(0.2, w.getWeather(), 0.0001);

        w = new Weather(new WeatherSettings());
        assertTrue(w.getNextWeatherChange() > 0);
    }
}
