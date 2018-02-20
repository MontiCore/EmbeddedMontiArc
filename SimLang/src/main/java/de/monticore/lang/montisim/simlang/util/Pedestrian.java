package de.monticore.lang.montisim.simlang.util;

public class Pedestrian {
  private float startX;
  private float startY;
  private float destX;
  private float destY;

  public Pedestrian(float startX, float startY, float destX, float destY) {
    this.startX = startX;
    this.startY = startY;
    this.destX = destX;
    this.destY = destY;
  }

  public float getStartX() {
    return startX;
  }

  public float getStartY() {
    return startY;
  }

  public float getDestX() {
    return destX;
  }

  public float getDestY() {
    return destY;
  }
}
