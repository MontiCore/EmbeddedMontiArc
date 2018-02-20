package de.monticore.lang.montisim.simlang.adapter;

import de.se_rwth.commons.logging.Log;
import de.monticore.lang.montisim.simlang.util.ConcreteWeather;
import de.monticore.lang.montisim.simlang.util.ExplicitVehicle;
import de.monticore.lang.montisim.simlang.util.Weather;

import de.monticore.lang.montisim.simlang.stolen.*;
//import simulation.environment.weather.Weather;
//import simulation.environment.weather.WeatherSettings;

import java.util.ArrayList;
import java.util.Optional;

public class SLMontiSimAdapter {
  private Optional<ArrayList<ExplicitVehicle>> vehicles;
  private de.monticore.lang.montisim.simlang.stolen.Weather weather;
  private String mapName;

  public SLMontiSimAdapter(GeneralSLAdapter adapter) {
    if(adapter.getMapName().isPresent()) {
      this.mapName = adapter.getMapName().get();
    } else {
      Log.error("No map name found.");
    }

    if(adapter.getWeather().isPresent()) {
     switch(adapter.getWeather().get().getType()) {
       case FIXED:
         boolean isSunny = adapter.getWeather().get().getWeatherObjects().get(0).getPrecipitationtype().equals(de.monticore.lang.montisim.simlang.util.SimLangEnums.PrecipitationTypes.NONE);
         if(isSunny) {
           this.weather = new de.monticore.lang.montisim.simlang.stolen.Weather(new WeatherSettings((double) 0.25));
         } else {
           this.weather = new de.monticore.lang.montisim.simlang.stolen.Weather(new WeatherSettings((double) 0.75));
         }
         break;
       case RANDOM:
         new de.monticore.lang.montisim.simlang.stolen.Weather(new WeatherSettings());
         break;
       default:
         Log.error("Cannot compute weather type.");
     }
    } else {
      //apply some default weather
      this.weather = new de.monticore.lang.montisim.simlang.stolen.Weather(new WeatherSettings((double) 0.25));
    }

    if(adapter.getExplicitVehicles().isPresent()) {
      this.vehicles = adapter.getExplicitVehicles();
    } else {
      this.vehicles = Optional.empty();
    }
  }

  public Optional<ArrayList<ExplicitVehicle>> getVehicles() {
    return vehicles;
  }

  public de.monticore.lang.montisim.simlang.stolen.Weather getWeather() {
    return weather;
  }

  public String getMapName() {
    return mapName;
  }
}
