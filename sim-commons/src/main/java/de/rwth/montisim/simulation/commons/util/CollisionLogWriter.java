package de.rwth.montisim.simulation.commons.util;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import de.rwth.montisim.commons.utils.Triplet;

//This class provides methods for logging of collisions
public class CollisionLogWriter{
    
    public static void addCollisions(LinkedHashMap<Triplet<String,String,Duration>,Duration> collisions, Instant startTime){
        addCollisions(collisions, startTime, false, false, 0);
    }
    
    public static void addCollisions(LinkedHashMap<Triplet<String,String,Duration>,Duration> collisions, Instant startTime, int episodeCounter){
        addCollisions(collisions, startTime, false, false, episodeCounter);
    }
    
    public static void addCollisions(LinkedHashMap<Triplet<String,String,Duration>,Duration> collisions, Instant startTime, boolean rl){
        addCollisions(collisions, startTime, rl, false, 0);
    }
    
    public static void addCollisions(LinkedHashMap<Triplet<String,String,Duration>,Duration> collisions, Instant startTime, boolean rl, boolean training){
        addCollisions(collisions, startTime, rl, training, 0);
    }
    
    public static void addCollisions(LinkedHashMap<Triplet<String,String,Duration>,Duration> collisions, Instant startTime, boolean rl, boolean training, int episodeCounter){
        try{
            Path outputFile;
            if(rl){
                if(training){
                    outputFile = Paths.get("results" + File.separator + "collision_log_training_" + startTime.toString().replaceAll(":","-") + ".csv");
                } else{
                    outputFile = Paths.get("results" + File.separator + "collision_log_execute_" + startTime.toString().replaceAll(":","-") + ".csv");
                }
                List<String> output = new ArrayList<>();

                //create file if it does not exist, append otherwise
                if(!Files.exists(outputFile)){
                    output.add("EpisodeNr,CrashParticipant1,CrashParticipant2,CrashTimeStamp,CrashDuration");

                    for(Triplet<String,String,Duration> x : collisions.keySet()){
                        output.add("" + episodeCounter + "," + x.getAt(0) + ",\"" + x.getAt(1) + "\"," + ((Duration) x.getAt(2)).toMillis()/1000.0 + "," + collisions.get(x).toMillis()/1000.0);
                    }

                    Files.write(outputFile, output);
                    return;
                }
                else{
                    for(Triplet<String,String,Duration> x : collisions.keySet()){
                        output.add("" + episodeCounter + "," + x.getAt(0) + ",\"" + x.getAt(1) + "\"," + ((Duration) x.getAt(2)).toMillis()/1000.0 + "," + collisions.get(x).toMillis()/1000.0);
                    }

                    Files.write(outputFile, output, StandardOpenOption.APPEND);
                    return;
                    
                }
            }
            else{
                outputFile = Paths.get("results" + File.separator + "collision_log_" + startTime.toString().replaceAll(":","-") + ".csv");

                //create file if it does not exist, append otherwise
                List<String> output = new ArrayList<>();
                if(!Files.exists(outputFile)){
                    output.add("EpisodeNr,CrashParticipant1,CrashParticipant2,CrashTimeStamp,CrashDuration");

                    for(Triplet<String,String,Duration> x : collisions.keySet()){
                        output.add("" + episodeCounter + "," + x.getAt(0) + ",\"" + x.getAt(1) + "\"," + ((Duration) x.getAt(2)).toMillis()/1000.0 + "," + collisions.get(x).toMillis()/1000.0);
                    }

                    Files.write(outputFile, output);
                    return;
                }
            }
        } catch (IOException e){
            e.printStackTrace();
            return;
       }
    }
}