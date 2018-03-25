package de.monticore.lang.montisim.simlang.util;

import de.monticore.lang.montisim.weather.cocos.NumberUnit;

import java.util.Optional;

public class RandomWeather {
  private Optional<NumberUnit> duration = Optional.empty();

  public RandomWeather() {}
  public RandomWeather(NumberUnit duration) {
    this.duration = Optional.of(duration);
  }

  public Optional<NumberUnit> getDuration() {
    return duration;
  }
}
