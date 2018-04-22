package de.monticore.lang.montisim.util.types;

import java.util.Optional;

public class Pedestrian {
  private float startX;
  private float startY;
  private Optional<Float> startZ = Optional.empty();
  private float destX;
  private float destY;
  private Optional<Float> destZ = Optional.empty();

  public Pedestrian(float startX, float startY, float destX, float destY) {
    this.startX = startX;
    this.startY = startY;
    this.destX = destX;
    this.destY = destY;
  }
  public Pedestrian(float startX, float startY, float destX, float destY, Float startZ, Float destZ) {
    this.startX = startX;
    this.startY = startY;
    this.destX = destX;
    this.destY = destY;

    if(startZ != null) {
      this.startZ = Optional.of(startZ);
    }
    if(destZ != null) {
      this.destZ = Optional.of(destZ);
    }
  }

  public float getStartX() {
    return startX;
  }

  public float getStartY() {
    return startY;
  }

  public Optional<Float> getStartZ() {
    return startZ;
  }

  public float getDestX() {
    return destX;
  }

  public float getDestY() {
    return destY;
  }

  public Optional<Float> getDestZ() {
    return destZ;
  }
}
