/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

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
