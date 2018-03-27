package de.monticore.lang.montisim.simlang.util;

import de.monticore.lang.montisim.weather.cocos.NumberUnit;

import java.util.ArrayList;

public class SequenceWeather {
  private ArrayList<ConcreteWeather> weathers;
  private ArrayList<NumberUnit> durations;

  public SequenceWeather(ArrayList weathers, ArrayList durations) {
    this.weathers = weathers;
    this.durations = durations;
  }

  public ArrayList<ConcreteWeather> getWeathers() {
    return weathers;
  }

  public ArrayList<NumberUnit> getDurations() {
    return durations;
  }
}
