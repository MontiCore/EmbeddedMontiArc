package de.monticore.lang.montisim.util.types;

import java.awt.geom.Point2D;

public class Path2D {
  private Point2D.Float start;
  private Point2D.Float dest;

  public Path2D(float startX, float startY, float destX, float destY) {
    this.start = new Point2D.Float(startX,startY);
    this.dest = new Point2D.Float(destX,destY);
  }

  public Point2D.Float getStart() {
    return start;
  }

  public Point2D.Float getDest() {
    return dest;
  }

  public float getStartX() {
    return this.start.x;
  }
  public float getStartY() {
    return this.start.y;
  }
  public float getDestX() {
    return this.dest.x;
  }
  public float getDestY() {
    return this.dest.y;
  }
}
