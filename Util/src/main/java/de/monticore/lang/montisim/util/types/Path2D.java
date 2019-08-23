/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import java.awt.geom.Point2D;

public class Path2D {
  private Point2D.Float start;
  private Point2D.Float dest;

  public Path2D(float startLat, float startLong, float destLat, float destLong) {
    this.start = new Point2D.Float(startLat,startLong);
    this.dest = new Point2D.Float(destLat,destLong);
  }

  public Point2D.Float getStart() {
    return start;
  }

  public Point2D.Float getDest() {
    return dest;
  }

  public float getStartLat() {
    return this.start.x;
  }
  public float getStartLong() {
    return this.start.y;
  }
  public float getDestLat() {
    return this.dest.x;
  }
  public float getDestLong() {
    return this.dest.y;
  }
}
