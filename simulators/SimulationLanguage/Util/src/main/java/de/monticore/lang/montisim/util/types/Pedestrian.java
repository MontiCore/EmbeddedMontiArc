/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import java.util.Optional;

public class Pedestrian {
  private float startLat;
  private float startLong;
  private Optional<Float> startZ = Optional.empty();
  private float destLat;
  private float destLong;
  private Optional<Float> destZ = Optional.empty();

  public Pedestrian(float startLat, float startLong, float destLat, float destLong) {
    this.startLat = startLat;
    this.startLong = startLong;
    this.destLat = destLat;
    this.destLong = destLong;
  }
  public Pedestrian(float startLat, float startLong, float destLat, float destLong, Float startZ, Float destZ) {
    this.startLat = startLat;
    this.startLong = startLong;
    this.destLat = destLat;
    this.destLong = destLong;

    if(startZ != null) {
      this.startZ = Optional.of(startZ);
    }
    if(destZ != null) {
      this.destZ = Optional.of(destZ);
    }
  }

  public float getStartLat() {
    return startLat;
  }

  public float getStartLong() {
    return startLong;
  }

  public Optional<Float> getStartZ() {
    return startZ;
  }

  public float getDestLat() {
    return destLat;
  }

  public float getDestLong() {
    return destLong;
  }

  public Optional<Float> getDestZ() {
    return destZ;
  }
}
