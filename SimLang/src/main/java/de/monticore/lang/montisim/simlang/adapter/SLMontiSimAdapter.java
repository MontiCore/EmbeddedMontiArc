/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang.adapter;

import de.se_rwth.commons.logging.Log;
import de.monticore.lang.montisim.util.types.ExplicitVehicle;

import simulation.environment.weather.Weather;
import simulation.environment.weather.WeatherSettings;

import java.util.ArrayList;
import java.util.Optional;

public class SLMontiSimAdapter {
  private Optional<ArrayList<ExplicitVehicle>> vehicles;
  private simulation.environment.weather.Weather weather;
  private String mapName;

  public SLMontiSimAdapter(SimLangContainer adapter) {
    if(adapter.getMapName().isPresent()) {
      this.mapName = adapter.getMapName().get();
    } else {
      Log.warn("No map name found.");
    }

    if(adapter.getWeather().isPresent()) {
     if(adapter.getWeather().get().get(0).getFixedWeather().isPresent()) {
       boolean isSunny = adapter.getWeather().get().get(0).getFixedWeather().get().getWeather().getPrecipitationType().equals(de.monticore.lang.montisim.util.types.SimLangEnums.PrecipitationTypes.NONE);
       if (isSunny) {
         this.weather = new Weather(new WeatherSettings(0.25d));
       } else {
         this.weather = new Weather(new WeatherSettings(0.75d));
       }
     }
     else if(adapter.getWeather().get().get(0).getRandomWeather().isPresent())
       new Weather(new WeatherSettings());
     else {
       Log.warn("Cannot compute weather type.");
     }
    } else {
      //apply some default weather
      this.weather = new Weather(new WeatherSettings(0.25d));
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

  public Weather getWeather() {
    return weather;
  }

  public String getMapName() {
    return mapName;
  }
}
