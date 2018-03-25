package de.monticore.lang.montisim.simlang.util;

import java.util.Optional;

public class RandomVehicle {
  private float amount;
  private Optional<Path2D> path = Optional.empty();

  public RandomVehicle(float amount) {
    this.amount = amount;
  }
  public RandomVehicle(float amount, Float startX, Float startY, Float destX, Float destY) {
    this.amount = amount;
    if(startX != null) {
      this.path = Optional.of(new Path2D(startX, startY, destX, destY));
    }
  }

  public float getAmount() {
    return amount;
  }

  public Optional getPath() {
    return path;
  }
}
