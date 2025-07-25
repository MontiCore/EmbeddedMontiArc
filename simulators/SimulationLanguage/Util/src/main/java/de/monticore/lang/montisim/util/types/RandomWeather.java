/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

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
