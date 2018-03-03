package de.monticore.lang.montisim.simlang.util;

import java.util.Optional;

public class Area {
  private Coordinate point1;
  private Optional<Coordinate> point2 = Optional.empty();
  private Optional<Float> radius = Optional.empty();

  public Area(Coordinate point1, Coordinate point2) {
    this.point1 = point1;
    this.point2 = Optional.of(point2);
  }
  public Area(Coordinate point1, float radius) {
    this.point1 = point1;
    this.radius = Optional.of(radius);
  }
  public Area(Coordinate point1, Optional opt) {
    this.point1 = point1;
    if(opt.get() instanceof Coordinate) {

    }
  }

  public boolean isRectangular() {
    return this.point2.isPresent();
  }
  public boolean isCircular() {
    return this.radius.isPresent();
  }

  public Coordinate getPoint1() {
    return point1;
  }

  public Optional<Coordinate> getPoint2() {
    return point2;
  }

  public Optional<Float> getRadius() {
    return radius;
  }
}
