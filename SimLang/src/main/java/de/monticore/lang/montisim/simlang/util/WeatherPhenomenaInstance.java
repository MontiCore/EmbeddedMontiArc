package de.monticore.lang.montisim.simlang.util;

import java.util.Optional;

public class WeatherPhenomenaInstance {
  private SimLangEnums.WeatherPhenomenas phenomena;
  private Optional<Coordinate> coord;

  public WeatherPhenomenaInstance(SimLangEnums.WeatherPhenomenas phenomena, Coordinate coord) {
    this.phenomena = phenomena;
    this.coord = Optional.of(coord);
  }

  public WeatherPhenomenaInstance(SimLangEnums.WeatherPhenomenas phenomena) {
    this.phenomena = phenomena;
  }

  public SimLangEnums.WeatherPhenomenas getPhenomena() {
    return phenomena;
  }

  public Optional<Coordinate> getCoord() {
    return coord;
  }
}
