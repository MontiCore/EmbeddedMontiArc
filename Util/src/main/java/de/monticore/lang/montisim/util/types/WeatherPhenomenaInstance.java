/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import java.awt.geom.Point2D;
import java.util.Optional;

public class WeatherPhenomenaInstance {
  private SimLangEnums.WeatherPhenomenas phenomena;
  private Optional<Point2D.Float> coord;

  public WeatherPhenomenaInstance(SimLangEnums.WeatherPhenomenas phenomena, Point2D.Float coord) {
    this.phenomena = phenomena;
    this.coord = Optional.of(coord);
  }
  public WeatherPhenomenaInstance(SimLangEnums.WeatherPhenomenas phenomena, float x, float y) {
    this.phenomena = phenomena;
    this.coord = Optional.of(new Point2D.Float(x,y));
  }

  public WeatherPhenomenaInstance(SimLangEnums.WeatherPhenomenas phenomena) {
    this.phenomena = phenomena;
  }

  public SimLangEnums.WeatherPhenomenas getPhenomena() {
    return phenomena;
  }

  public Optional<Point2D.Float> getCoord() {
    return coord;
  }
}
