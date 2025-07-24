/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

public class FixedWeather {
  private ConcreteWeather weather;

  public FixedWeather(ConcreteWeather weather) {
    this.weather = weather;
  }

  public ConcreteWeather getWeather() {
    return weather;
  }
}
