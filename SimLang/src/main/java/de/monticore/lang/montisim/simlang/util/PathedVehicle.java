package de.monticore.lang.montisim.simlang.util;

public class PathedVehicle {
  private Float startX;
  private Float startY;
  private Float startRad;
  private Float destX;
  private Float destY;
  private Float destRad;
  private Float amount;

  public PathedVehicle(Float startX, Float startY, Float startRad, Float destX, Float destY, Float destRad, Float amount) {
    this.startX = startX;
    this.startY = startY;
    this.startRad = startRad;
    this.destX = destX;
    this.destY = destY;
    this.destRad = destRad;
    this.amount = amount;
  }

  public Float getStartX() {
    return startX;
  }

  public Float getStartY() {
    return startY;
  }

  public Float getStartRad() {
    return startRad;
  }

  public Float getDestX() {
    return destX;
  }

  public Float getDestY() {
    return destY;
  }

  public Float getDestRad() {
    return destRad;
  }

  public Float getAmount() {
    return amount;
  }
}
