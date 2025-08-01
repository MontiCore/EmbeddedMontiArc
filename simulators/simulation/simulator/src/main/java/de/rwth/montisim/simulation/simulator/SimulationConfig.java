/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.eecomponents.simple_network.ModuleProperties;
import de.rwth.montisim.simulation.eecomponents.simple_network.SimulatorModule;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMissingComponentException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.communication.PreprocessorProperties;
import de.rwth.montisim.simulation.simulator.randomization.RandomizationProperties;
import de.rwth.montisim.simulation.simulator.rewards.RewardFunctionProperties;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;

import java.io.File;
import java.time.Duration;
import java.time.Instant;
import java.util.Objects;
import java.util.Optional;
import java.util.Vector;

public class SimulationConfig {
    public String name;
    public String map_name;
    public Duration max_duration = Duration.ofSeconds(60);
    public Duration tick_duration = Duration.ofNanos(Time.SECOND_TO_NANOSEC / 100);
    public Instant start_time = Instant.now();
    public Vector<RandomizationProperties> randomization = new Vector<>();
    public Optional<PreprocessorProperties> preprocessor = Optional.empty();
    public Optional<RewardFunctionProperties> rewardFunction = Optional.empty();

    public Vector<VehicleProperties> cars = new Vector<>();
    public Vector<ModuleProperties> modules = new Vector<>();
    public String collision_mode = "IGNORE_COLLISIONS";

    public static SimulationConfig fromFile(File file) throws SerializationException {
        return Json.instantiateFromJson(file, SimulationConfig.class);
    }

    public Simulator build(World world, Pathfinding pathfinding, OsmMap map) {
        Objects.requireNonNull(world);
        Objects.requireNonNull(pathfinding);
        Simulator sim = new Simulator(this, world, pathfinding, map);

        for (ModuleProperties mod : modules) {
            SimulatorModule m = mod.build(sim.buildContext);
            sim.modules.add(m);
            sim.moduleByName.put(mod.getName(), m);
            sim.buildContext.addObject(m);
        }
        // Add Vehicles
        for (VehicleProperties v : cars) {
            Vehicle vehicle;
            try {
                vehicle = sim.getVehicleBuilder(v).build();
            } catch (SerializationException | EEMessageTypeException | EESetupException
                     | EEMissingComponentException e) {
                throw new IllegalStateException(e);
            }
            sim.addSimulationObject(vehicle);
        }
        return sim;
    }
}