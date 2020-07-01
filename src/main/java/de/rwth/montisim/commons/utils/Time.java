/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.utils.JsonTraverser.ArrayEntryStream;

public class Time {
    public static final long SECOND_TO_NANOSEC = 1000000000;
    public static final double NANOSEC_TO_SEC = 0.000000001;
    
    public static double secondsFromDuration(Duration d){
        return d.getSeconds() + d.getNano()*NANOSEC_TO_SEC;
    }
    public static long nanosecondsFromDuration(Duration d){
        return d.getNano() + d.getSeconds()*SECOND_TO_NANOSEC;
    }
    public static Duration durationFromSeconds(double seconds){
        return Duration.ofNanos((long)(seconds*SECOND_TO_NANOSEC));
    }

    public static void toJson(JsonWriter j, Instant t) {
        j.startArray();
        j.writeValue(t.getEpochSecond());
        j.writeValue(t.getNano());
        j.endArray();
    }

    public static Instant fromJson(JsonTraverser j){
        ArrayEntryStream it = j.streamArray().it;
        if (!it.hasNext()) throw new ParsingException("Expected 'Seconds' entry");
        it.next();
        long sec = j.getLong();
        if (!it.hasNext()) throw new ParsingException("Expected 'nanoseconds' entry");
        it.next();
        long nano = j.getLong();
        Instant t = Instant.ofEpochSecond(sec, nano);
        return t;
    }
}