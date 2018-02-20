package de.monticore.lang.montisim.simlang.util;

public class ExplicitVehicle {
  private Float startX;
  private Float startY;
  private Float startRot;
  private Float destX;
  private Float destY;
  private String name;

  public ExplicitVehicle(String name, Float startX, Float startY, Float startRot, Float destX, Float destY) {
    this.startX = startX;
    this.startY = startY;
    this.startRot = startRot;
    this.destX = destX;
    this.destY = destY;
    this.name = name;
  }

  public Float getStartX() {
    return startX;
  }

  public Float getStartY() {
    return startY;
  }

  public Float getStartRot() {
    return startRot;
  }

  public Float getDestX() {
    return destX;
  }

  public Float getDestY() {
    return destY;
  }

  public String getName() {
    return name;
  }
}
