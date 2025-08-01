package de.rwth.montisim.simulation.commons.util;

import java.time.Duration;

public class VelocityLogEntry {

    private final String vehicleName;
    private final double velocity;
    private final Duration timeStamp;
    private final int stepCounter;

    public VelocityLogEntry(String vehicleName, int stepCounter, Double velocity, Duration timeStamp) {
        this.vehicleName = vehicleName;
        this.stepCounter = stepCounter;
        this.velocity = velocity;
        this.timeStamp = timeStamp;
    }

    public static String getCSVHeader() {
        return "Episode,Step,VehicleName,Velocity,TimeStamp";
    }

    public String getVehicleName() {
        return vehicleName;
    }

    public double getVelocity() {
        return velocity;
    }

    public Duration getTimeStamp() {
        return timeStamp;
    }

    @Override
    public String toString() {
      return stepCounter + ",\"" + vehicleName + "\",\"" + velocity + "\"," + (timeStamp.toMillis() / 1000.0);
    }

    public String toCSV(int episodeCounter) {
        return episodeCounter + "," + this;
    }

}
