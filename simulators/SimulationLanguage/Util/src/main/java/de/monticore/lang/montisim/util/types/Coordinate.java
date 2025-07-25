/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

public class Coordinate {
  private float latitude;
  private float longitude;

  public Coordinate(float latitude, float longitude) {
    this.latitude = latitude;
    this.longitude = longitude;
  }

  public float getLatitude() {
    return latitude;
  }

  public float getLongitude() {
    return longitude;
  }
}
