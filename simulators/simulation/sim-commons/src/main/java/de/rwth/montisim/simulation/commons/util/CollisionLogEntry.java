package de.rwth.montisim.simulation.commons.util;

import java.time.Duration;

public class CollisionLogEntry {

  private final String vehicleName;
  private final String collisionObjectName;
  private final Duration timeStamp;
  CollisionType collisionType;
  private Duration duration;

  public CollisionLogEntry(CollisionType collisionType, String vehicleName, String collisionObjectName, Duration timeStamp) {
    this.collisionType = collisionType;
    this.vehicleName = vehicleName;
    this.collisionObjectName = collisionObjectName;
    this.timeStamp = timeStamp;
    this.duration = Duration.ZERO;
  }

  public static String getCSVHeader() {
    return "Episode,CollisionType,VehicleName,CollisionObjectName,TimeStamp,Duration";
  }

  public String getVehicleName() {
    return vehicleName;
  }

  public String getCollisionObjectName() {
    return collisionObjectName;
  }

  public CollisionType getCollisionType() {
    return collisionType;
  }

  public Duration getTimeStamp() {
    return timeStamp;
  }

  public Duration getDuration() {
    return duration;
  }

  public void setDuration(Duration duration) {
    this.duration = duration;
  }

  public void addDuration(Duration duration) {
    this.duration = this.duration.plus(duration);
  }

  @Override
  public String toString() {
    return collisionType + ",\"" + vehicleName + "\",\"" + collisionObjectName + "\"," + (timeStamp.toMillis() / 1000.0) + "," + (duration.toMillis() / 1000.0);
  }

  public String toCSV(int episodeCounter) {
    return episodeCounter + "," + this;
  }

  public enum CollisionType {
    STATIC_COLLISION("static"), VEHICLE_COLLISION("vehicle");

    private final String type;

    CollisionType(String type) {
      this.type = type;
    }

    public String getType() {
      return type;
    }

    @Override
    public String toString() {
      return type;
    }
  }
}
