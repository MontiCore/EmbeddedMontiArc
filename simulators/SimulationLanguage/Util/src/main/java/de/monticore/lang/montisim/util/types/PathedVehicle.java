/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import java.util.Optional;

public class PathedVehicle {
  private Path2D path;
  private NumberUnit startRad;
  private NumberUnit destRad;
  private Optional<Float> amount = Optional.empty();

  public PathedVehicle(float startLat, float startLong, NumberUnit startRad, float destLat, float destLong, NumberUnit destRad) {
    this.path = new Path2D(startLat, startLong, destLat, destLong);
    this.startRad = startRad;
    this.destRad = destRad;
  }
  public PathedVehicle(float startLat, float startLong, NumberUnit startRad, float destLat, float destLong, NumberUnit destRad, Float amount) {
    this.path = new Path2D(startLat, startLong, destLat, destLong);
    this.startRad = startRad;
    this.destRad = destRad;
    if(amount != null) {
      this.amount = Optional.of(amount);
    }
  }

  public Path2D getPath() {
    return path;
  }

  public NumberUnit getStartRad() {
    return startRad;
  }

  public NumberUnit getDestRad() {
    return destRad;
  }

  public Optional<Float> getAmount() {
    return amount;
  }
}
