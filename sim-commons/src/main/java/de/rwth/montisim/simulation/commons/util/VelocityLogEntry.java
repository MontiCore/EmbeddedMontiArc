package de.rwth.montisim.simulation.commons.util;

import java.time.Duration;

public class VelocityLogEntry {

    private final String vehicleName;
    private final double velocity;
    private final Duration timeStamp;

    public VelocityLogEntry(String vehicleName, Double velocity, Duration timeStamp) {
        this.vehicleName = vehicleName;
        this.velocity = velocity;
        this.timeStamp = timeStamp;
    }

    public static String getCSVHeader() {
        return "Episode,VehicleName,Velocity,TimeStamp";
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
        return vehicleName + "\",\"" + velocity + "\"," + (timeStamp.toMillis() / 1000.0);
    }

    public String toCSV(int episodeCounter) {
        return episodeCounter + "," + this;
    }

}
