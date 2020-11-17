/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang.adapter;

import de.monticore.ast.ASTNode;
import de.monticore.lang.montisim.simlang._ast.*;
import de.monticore.lang.montisim.simlang._symboltable.*;
import de.monticore.lang.montisim.simlang._visitor.SimLangInheritanceVisitor;
import de.monticore.lang.montisim.simlang._visitor.SimLangVisitor;
import de.monticore.lang.montisim.util.types.*;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;

public class SimLangContainer {

    // TODO: don't use Optionals as fields
    private Optional<AlternativeInput> sim_render_frequency = Optional.empty();
    private Optional<AlternativeInput> sim_loop_frequency = Optional.empty();
    private Optional<AlternativeInput> sim_duration = Optional.empty();
    private Optional<SimLangEnums.SimulationTypes> sim_type = Optional.empty();
    private Optional<ArrayList<Weather>> weather = Optional.empty();
    private Optional<ArrayList<Time>> time = Optional.empty();
    private Optional<String> map_path = Optional.empty();
    private Optional<String> map_name = Optional.empty();
    private Optional<MapHeight> map_height = Optional.empty();
    private Optional<AlternativeInput> map_overlap = Optional.empty();
    private Optional<AlternativeInput> map_sector_width = Optional.empty();
    private Optional<AlternativeInput> map_sector_height = Optional.empty();
    private Optional<AlternativeInput> max_sector_users = Optional.empty();
    private Optional<AlternativeInput> timeout = Optional.empty();
    private Optional<AlternativeInput> gravity = Optional.empty();
    private Optional<AlternativeInput> pedestrian_density = Optional.empty();

    private Optional<ArrayList<Pedestrian>> pedestrians = Optional.empty();

    private Optional<ArrayList<ExplicitVehicle>> explicit_vehicles = Optional.empty();
    private Optional<ArrayList<RandomVehicle>> random_vehicles = Optional.empty();
    private Optional<ArrayList<PathedVehicle>> pathed_vehicles = Optional.empty();
    private Optional<ArrayList<LTLVehicle>> ltl_vehicles = Optional.empty();

    private Optional<ArrayList<Channel>> channels = Optional.empty();

    // TODO: replace with visitor and get rid of unnecessary symbols (but then we will also need to change the cocos)
    public SimLangContainer(Scope symTab, String fullPackage, String modelName) {
        String symPrefix = fullPackage.equals("") ? modelName + "." : fullPackage + "." + modelName + ".";
        SimulationRenderFrequencySymbol sim_render_frequency = symTab.<SimulationRenderFrequencySymbol>resolve(symPrefix + "sim_render_frequency", SimulationRenderFrequencySymbol.KIND).orElse(null);
        if (sim_render_frequency != null)
            this.sim_render_frequency = Optional.of(sim_render_frequency.getSimulationRenderFrequency());

        SimulationLoopFrequencySymbol sim_loop_frequency = symTab.<SimulationLoopFrequencySymbol>resolve(symPrefix + "sim_loop_frequency", SimulationLoopFrequencySymbol.KIND).orElse(null);
        if (sim_loop_frequency != null)
            this.sim_loop_frequency = Optional.of(sim_loop_frequency.getSimulationLoopFrequency());

        SimulationDurationSymbol sim_duration = symTab.<SimulationDurationSymbol>resolve(symPrefix + "sim_duration", SimulationDurationSymbol.KIND).orElse(null);
        if (sim_duration != null)
            this.sim_duration = Optional.of(sim_duration.getSimulationDuration());

        SimulationTypeSymbol sim_type = symTab.<SimulationTypeSymbol>resolve(symPrefix + "sim_type", SimulationTypeSymbol.KIND).orElse(null);
        if (sim_type != null)
            this.sim_type = Optional.of(sim_type.getSimulationType());

        TimeSymbol time = symTab.<TimeSymbol>resolve(symPrefix + "time", TimeSymbol.KIND).orElse(null);
        if (time != null)
            this.time = Optional.of(time.getTime());

        WeatherSymbol weather = symTab.<WeatherSymbol>resolve(symPrefix + "weather", WeatherSymbol.KIND).orElse(null);
        if (weather != null)
            this.weather = Optional.of(weather.getWeathers());

        MapPathSymbol map_path = symTab.<MapPathSymbol>resolve(symPrefix + "map_path", MapPathSymbol.KIND).orElse(null);
        if (map_path != null)
            this.map_path = Optional.of(map_path.getMapPath());

        MapNameSymbol map_name = symTab.<MapNameSymbol>resolve(symPrefix + "map_name", MapNameSymbol.KIND).orElse(null);
        if (map_name != null)
            this.map_name = Optional.of(map_name.getMapName());

        MapHeightSymbol map_height = symTab.<MapHeightSymbol>resolve(symPrefix + "map_height", MapHeightSymbol.KIND).orElse(null);
        if (map_height != null)
            this.map_height = Optional.of(map_height.getMapHeight());

        MapOverlapSymbol map_overlap = symTab.<MapOverlapSymbol>resolve(symPrefix + "map_overlap", MapOverlapSymbol.KIND).orElse(null);
        if (sim_render_frequency != null)
            this.map_overlap = Optional.of(map_overlap.getMapOverlap());

        MapSectorWidthSymbol map_sector_width = symTab.<MapSectorWidthSymbol>resolve(symPrefix + "map_sector_width", MapSectorWidthSymbol.KIND).orElse(null);
        if (map_sector_width != null)
            this.map_sector_width = Optional.of(map_sector_width.getMapSectorWidth());

        MapSectorHeightSymbol map_sector_height = symTab.<MapSectorHeightSymbol>resolve(symPrefix + "map_sector_height", MapSectorHeightSymbol.KIND).orElse(null);
        if (map_sector_height != null)
            this.map_sector_height = Optional.of(map_sector_height.getMapSectorHeight());

        MaxSectorUsersSymbol max_sector_users = symTab.<MaxSectorUsersSymbol>resolve(symPrefix + "max_sector_users", MaxSectorUsersSymbol.KIND).orElse(null);
        if (max_sector_users != null)
            this.max_sector_users = Optional.of(max_sector_users.getMaxSectorUsers());

        TimeoutSymbol timeout = symTab.<TimeoutSymbol>resolve(symPrefix + "timeout", TimeoutSymbol.KIND).orElse(null);
        if (timeout != null)
            this.timeout = Optional.of(timeout.getTimeout());

        GravitySymbol gravity = symTab.<GravitySymbol>resolve(symPrefix + "gravity", GravitySymbol.KIND).orElse(null);
        if (gravity != null)
            this.gravity = Optional.of(gravity.getGravity());

        PedestrianDensitySymbol pedestrian_density = symTab.<PedestrianDensitySymbol>resolve(symPrefix + "pedestrian_density", PedestrianDensitySymbol.KIND).orElse(null);
        if (pedestrian_density != null)
            this.pedestrian_density = Optional.of(pedestrian_density.getPedestrianDensity());

        Collection<PedestrianSymbol> pedestrians = symTab.<PedestrianSymbol>resolveMany(symPrefix + "pedestrian", PedestrianSymbol.KIND);
        if (!pedestrians.isEmpty()) {
            ArrayList<Pedestrian> content = new ArrayList<>();
            for (PedestrianSymbol sym : pedestrians) {
                content.add(sym.getPedestrian());
            }
            this.pedestrians = Optional.of(content);
        }

        Collection<ExplicitVehicleSymbol> explicit_vehicles = symTab.resolveMany(symPrefix + "explicit_vehicle", ExplicitVehicleSymbol.KIND);
        if (!explicit_vehicles.isEmpty()) {
            ArrayList<ExplicitVehicle> content = new ArrayList<>();
            for (ExplicitVehicleSymbol sym : explicit_vehicles) {
                content.add(sym.getVehicle());
            }
            this.explicit_vehicles = Optional.of(content);
        }

        Collection<PathedVehicleSymbol> pathed_vehicles = symTab.<PathedVehicleSymbol>resolveMany(symPrefix + "pathed_vehicle", PathedVehicleSymbol.KIND);
        if (!pathed_vehicles.isEmpty()) {
            ArrayList<PathedVehicle> content = new ArrayList<>();
            for (PathedVehicleSymbol sym : pathed_vehicles) {
                content.add(sym.getVehicle());
            }
            this.pathed_vehicles = Optional.of(content);
        }

        Collection<LTLVehicleSymbol> ltl_vehicles = symTab.resolveMany(symPrefix + "ltl_vehicle", LTLVehicleSymbol.KIND);
        if (!ltl_vehicles.isEmpty()) {
            ArrayList<LTLVehicle> content = new ArrayList<>();
            for (LTLVehicleSymbol sym : ltl_vehicles) {
                content.add(sym.getVehicle());
            }
            this.ltl_vehicles = Optional.of(content);
        }

        Collection<RandomVehicleSymbol> random_vehicles = symTab.<RandomVehicleSymbol>resolveMany(symPrefix + "random_vehicle", RandomVehicleSymbol.KIND);
        if (!random_vehicles.isEmpty()) {
            ArrayList<RandomVehicle> content = new ArrayList<>();
            for (RandomVehicleSymbol sym : random_vehicles) {
                content.add(sym.getVehicle());
            }
            this.random_vehicles = Optional.of(content);
        }

        Collection<ChannelSymbol> channels = symTab.<ChannelSymbol>resolveMany(symPrefix + "channel", ChannelSymbol.KIND);
        if (!channels.isEmpty()) {
            ArrayList<Channel> content = new ArrayList<>();
            for (ChannelSymbol sym : channels) {
                content.add(sym.getChannel());
            }
            this.channels = Optional.of(content);
        }
    }


    /*
     * GETTER METHODS
     *
     */
    public Optional<AlternativeInput> getSimulationRenderFrequency() {
        return this.sim_render_frequency;
    }

    public Optional<AlternativeInput> getSimulationLoopFrequency() {
        return this.sim_loop_frequency;
    }

    public Optional<AlternativeInput> getSimulationDuration() {
        return this.sim_duration;
    }

    public Optional<SimLangEnums.SimulationTypes> getSimulationType() {
        return this.sim_type;
    }

    public Optional<ArrayList<Weather>> getWeather() {
        return this.weather;
    }

    public Optional<ArrayList<Time>> getTime() {
        return this.time;
    }

    public Optional<String> getMapPath() {
        return this.map_path;
    }

    public Optional<String> getMapName() {
        return this.map_name;
    }

    public Optional<MapHeight> getMapHeight() {
        return this.map_height;
    }

    public Optional<AlternativeInput> getMapOverlap() {
        return this.map_overlap;
    }

    public Optional<AlternativeInput> getMapSectorWidth() {
        return this.map_sector_width;
    }

    public Optional<AlternativeInput> getMapSectorHeight() {
        return this.map_sector_height;
    }

    public Optional<AlternativeInput> getMaxSectorUsers() {
        return this.max_sector_users;
    }

    public Optional<AlternativeInput> getTimeout() {
        return this.timeout;
    }

    public Optional<AlternativeInput> getGravity() {
        return this.gravity;
    }

    public Optional<ArrayList<Pedestrian>> getPedestrians() {
        return this.pedestrians;
    }

    public Optional<AlternativeInput> getPedestrianDensity() {
        return this.pedestrian_density;
    }

    public Optional<ArrayList<ExplicitVehicle>> getExplicitVehicles() {
        return this.explicit_vehicles;
    }

    public Optional<ArrayList<PathedVehicle>> getPathedVehicles() {
        return this.pathed_vehicles;
    }

    public Optional<ArrayList<LTLVehicle>> getLTLVehicles() {
        return this.ltl_vehicles;
    }

    public Optional<ArrayList<RandomVehicle>> getRandomVehicles() {
        return this.random_vehicles;
    }

    public Optional<ArrayList<Channel>> getChannels() {
        return this.channels;
    }
}
