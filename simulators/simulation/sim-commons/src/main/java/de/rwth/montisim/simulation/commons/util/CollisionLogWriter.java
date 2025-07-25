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

//This class provides methods for logging of collisions
public class CollisionLogWriter {

  public static void addCollisions(Vector<CollisionLogEntry> collisions, Instant startTime) {
    addCollisions(collisions, startTime, false, false, 0);
  }

  public static void addCollisions(Vector<CollisionLogEntry> collisions, Instant startTime, int episodeCounter) {
    addCollisions(collisions, startTime, false, false, episodeCounter);
  }

  public static void addCollisions(Vector<CollisionLogEntry> collisions, Instant startTime, boolean rl) {
    addCollisions(collisions, startTime, rl, false, 0);
  }

  public static void addCollisions(Vector<CollisionLogEntry> collisions, Instant startTime, boolean rl, boolean training) {
    addCollisions(collisions, startTime, rl, training, 0);
  }

  public static void addCollisions(Vector<CollisionLogEntry> collisions, Instant startTime, boolean rl, boolean training, int episodeCounter) {
    try {
      Path outputFile;
      if (rl) {
        if (training) {
          outputFile = Paths.get("results" + File.separator + "collision_log_training_" + startTime.toString().replaceAll(":", "-") + ".csv");
        }
        else {
          outputFile = Paths.get("results" + File.separator + "collision_log_execute_" + startTime.toString().replaceAll(":", "-") + ".csv");
        }
        List<String> output = new ArrayList<>();

        //create file if it does not exist, append otherwise
        if (!Files.exists(outputFile)) {
          output.add(CollisionLogEntry.getCSVHeader());

          for (CollisionLogEntry x : collisions) {
            output.add(x.toCSV(episodeCounter));
          }

          Files.write(outputFile, output);
        }
        else {
          for (CollisionLogEntry x : collisions) {
            output.add(x.toCSV(episodeCounter));
          }

          Files.write(outputFile, output, StandardOpenOption.APPEND);

        }
      }
      else {
        outputFile = Paths.get("results" + File.separator + "collision_log_" + startTime.toString().replaceAll(":", "-") + ".csv");

        //create file if it does not exist, append otherwise
        List<String> output = new ArrayList<>();
        if (!Files.exists(outputFile)) {
          output.add(CollisionLogEntry.getCSVHeader());

          for (CollisionLogEntry x : collisions) {
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