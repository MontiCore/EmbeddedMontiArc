/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import java.util.Optional;

public class RandomVehicle {
  private float amount;
  private Optional<Path2D> path = Optional.empty();

  public RandomVehicle(float amount) {
    this.amount = amount;
  }
  public RandomVehicle(float amount, Float startLat, Float startLong, Float destLat, Float destLong) {
    this.amount = amount;
    if(startLat != null) {
      this.path = Optional.of(new Path2D(startLat, startLong, destLat, destLong));
    }
  }

  public float getAmount() {
    return amount;
  }

  public Optional<Path2D> getPath() {
    return path;
  }
}
