/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator;

import java.io.File;
import java.time.*;
import java.util.Vector;

import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;

public class SimulationConfig {
    public String name;
    public String map_name;
    public Duration max_duration = Duration.ofSeconds(60);
    public Duration tick_duration = Duration.ofNanos(Time.SECOND_TO_NANOSEC / 100);
    public Instant start_time = Instant.now();

    public Vector<VehicleProperties> cars = new Vector<>();

    public static SimulationConfig fromFile(File file) throws SerializationException {
        return Json.instantiateFromJson(file, SimulationConfig.class);
    }
}