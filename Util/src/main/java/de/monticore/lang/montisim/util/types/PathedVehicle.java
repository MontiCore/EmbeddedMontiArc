package de.monticore.lang.montisim.util.types;

import java.util.Optional;

public class PathedVehicle {
  private Path2D path;
  private NumberUnit startRad;
  private NumberUnit destRad;
  private Optional<Float> amount = Optional.empty();

  public PathedVehicle(float startX, float startY, NumberUnit startRad, float destX, float destY, NumberUnit destRad) {
    this.path = new Path2D(startX, startY, destX, destY);
    this.startRad = startRad;
    this.destRad = destRad;
  }
  public PathedVehicle(float startX, float startY, NumberUnit startRad, float destX, float destY, NumberUnit destRad, Float amount) {
    this.path = new Path2D(startX, startY, destX, destY);
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
