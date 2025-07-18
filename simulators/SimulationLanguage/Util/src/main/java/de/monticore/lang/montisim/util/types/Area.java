/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import java.awt.geom.Point2D;
import java.util.Optional;

public class Area {
  private Optional<Point2D.Float> point1 = Optional.empty();
  private Optional<Point2D.Float> point2 = Optional.empty();
  private Optional<NumberUnit> radius = Optional.empty();

  //global
  public Area() { }

  //rectangular
  public Area(Point2D.Float point1, Point2D.Float point2) {
    this.point1 = Optional.of(point1);
    this.point2 = Optional.of(point2);
  }

  //circular
  public Area(Point2D.Float point1, NumberUnit radius) {
    this.point1 = Optional.of(point1);
    this.radius = Optional.of(radius);
  }

  public boolean isGlobal() {return (!point1.isPresent()); }
  public boolean isRectangular() {
    return this.point2.isPresent();
  }
  public boolean isCircular() {
    return this.radius.isPresent();
  }

  public Optional<Point2D.Float> getPoint1() {
    return point1;
  }

  public Optional<Point2D.Float> getPoint2() {
    return point2;
  }

  public Optional<NumberUnit> getRadius() {
    return radius;
  }
}
