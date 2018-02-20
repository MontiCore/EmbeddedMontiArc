package de.monticore.lang.montisim.simlang.util;

public class RandomVehicle {
  private Float amount;
  private Float startX;
  private Float startY;
  private Float destX;
  private Float destY;


  public RandomVehicle(Float amount, Float startX, Float startY, Float destX, Float destY) {
    this.amount = amount;
    this.startX = startX;
    this.startY = startY;
    this.destX = destX;
    this.destY = destY;
  }

  public Float getAmount() {
    return amount;
  }

  public Float getStartX() {
    return startX;
  }

  public Float getStartY() {
    return startY;
  }

  public Float getDestX() {
    return destX;
  }

  public Float getDestY() {
    return destY;
  }
}
