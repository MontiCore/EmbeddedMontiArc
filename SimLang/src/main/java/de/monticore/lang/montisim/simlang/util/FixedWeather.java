package de.monticore.lang.montisim.simlang.util;

public class FixedWeather {
  private ConcreteWeather weather;

  public FixedWeather(ConcreteWeather weather) {
    this.weather = weather;
  }

  public ConcreteWeather getWeather() {
    return weather;
  }
}
