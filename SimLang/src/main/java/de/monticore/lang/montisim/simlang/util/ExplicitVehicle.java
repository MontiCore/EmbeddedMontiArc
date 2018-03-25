package de.monticore.lang.montisim.simlang.util;

import java.util.Optional;

public class ExplicitVehicle {
  private Path2D path;
  private float startRot;
  private Optional<Float> destZ = Optional.empty();
  private String name;

  public ExplicitVehicle(String name, float startX, float startY, float destX, float destY, float startRot) {
    this.path = new Path2D(startX, startY, destX, destY);
    this.startRot = startRot;
    this.name = name;
  }
  public ExplicitVehicle(String name, float startX, float startY, float destX, float destY, float startRot, Float destZ) {
    this.path = new Path2D(startX, startY, destX, destY);
    if(destZ != null) {
      this.destZ = Optional.of(destZ);
    }
    this.startRot = startRot;
    this.name = name;
  }

  public Path2D getPath() {
    return path;
  }

  public Float getStartRot() {
    return startRot;
  }

  public Optional<Float> getDestZ() {
    return destZ;
  }

  public String getName() {
    return name;
  }
}
