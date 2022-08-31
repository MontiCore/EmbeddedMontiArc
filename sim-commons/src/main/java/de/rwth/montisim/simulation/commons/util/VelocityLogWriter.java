package de.rwth.montisim.simulation.commons.util;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class VelocityLogWriter {

    public static void addVelocities(Vector<VelocityLogEntry> velocities, Instant startTime) {
        addVelocities(velocities, startTime, false, false, 0);
    }

    public static void addVelocities(Vector<VelocityLogEntry> velocities, Instant startTime, int episodeCounter) {
        addVelocities(velocities, startTime, false, false, episodeCounter);
    }

    public static void addVelocities(Vector<VelocityLogEntry> velocities, Instant startTime, boolean rl) {
        addVelocities(velocities, startTime, rl, false, 0);
    }

    public static void addVelocities(Vector<VelocityLogEntry> velocities, Instant startTime, boolean rl, boolean training) {
        addVelocities(velocities, startTime, rl, training, 0);
    }

    public static void addVelocities(Vector<VelocityLogEntry> velocities, Instant startTime, boolean rl, boolean training, int episodeCounter) {
        try {
            Path outputFile;
            if (rl) {
                if (training) {
                    outputFile = Paths.get("results" + File.separator + "velocity_log_training_" + startTime.toString().replaceAll(":", "-") + ".csv");
                }
                else {
                    outputFile = Paths.get("results" + File.separator + "velocity_log_execute_" + startTime.toString().replaceAll(":", "-") + ".csv");
                }
                List<String> output = new ArrayList<>();

                //create file if it does not exist, append otherwise
                if (!Files.exists(outputFile)) {
                    output.add(VelocityLogEntry.getCSVHeader());

                    for (VelocityLogEntry x : velocities) {
                        output.add(x.toCSV(episodeCounter));
                    }

                    Files.write(outputFile, output);
                }
                else {
                    for (VelocityLogEntry x : velocities) {
                        output.add(x.toCSV(episodeCounter));
                    }

                    Files.write(outputFile, output, StandardOpenOption.APPEND);

                }
            }
            else {
                outputFile = Paths.get("results" + File.separator + "velocity_log_" + startTime.toString().replaceAll(":", "-") + ".csv");

                //create file if it does not exist, append otherwise
                List<String> output = new ArrayList<>();
                if (!Files.exists(outputFile)) {
                    output.add(VelocityLogEntry.getCSVHeader());

                    for (VelocityLogEntry x : velocities) {
                        output.add(x.toCSV(episodeCounter));
                    }

                    Files.write(outputFile, output);
                }
            }
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }
}
