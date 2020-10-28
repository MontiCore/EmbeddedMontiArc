package de.rwth.montisim.simulation.simulator;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.environment.world.World;

import java.util.Objects;

public class SimulatorBuilder {
    boolean fromJson = false;
    SimulationConfig config;
    String jsonConfig;
    World world;
    Pathfinding pathfinding;
    MessageTypeManager mtManager;


    public static SimulatorBuilder fromJsonConfig(String jsonConfig) {
        SimulatorBuilder builder = new SimulatorBuilder();
        builder.fromJson = true;
        builder.jsonConfig = jsonConfig;
        return builder;
    }

    public static SimulatorBuilder withDefaultConfig() {
        SimulatorBuilder bd = new SimulatorBuilder();
        bd.config = new SimulationConfig();
        return bd;
    }

    public Simulator build() throws SerializationException {
        if (fromJson){
            Objects.requireNonNull(jsonConfig);
            config = Json.instantiateFromJson(jsonConfig, SimulationConfig.class);
            fromJson = false;
            return build();
        }

        // throw runtime exception if there are anything missing
        Objects.requireNonNull(config);
        Objects.requireNonNull(world);
        Objects.requireNonNull(pathfinding);
        Objects.requireNonNull(mtManager);
        return new Simulator(config, world, pathfinding, mtManager);
    }

    public SimulatorBuilder setConfig(SimulationConfig config) {
        this.config = config;
        return this;
    }

    public SimulatorBuilder setWorld(World world) {
        this.world = world;
        return this;
    }

    public SimulatorBuilder setPathfinding(Pathfinding pathfinding) {
        this.pathfinding = pathfinding;
        return this;
    }

    public SimulatorBuilder setMtManager(MessageTypeManager mtManager) {
        this.mtManager = mtManager;
        return this;
    }
}
