package de.monticore.lang.montisim.simlang.adapter;

import de.monticore.lang.montisim.simlang._symboltable.*;
import de.monticore.symboltable.Scope;
import de.monticore.lang.montisim.simlang.util.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Optional;

public class SimLangContainer {

  private Optional<AlternativeInput> sim_render_frequency = Optional.empty();
  private Optional<AlternativeInput> sim_loop_frequency;
  private Optional<AlternativeInput> sim_duration;
  private Optional<SimLangEnums.SimulationTypes> sim_type;
  private Optional<ArrayList<Weather>> weather;
  private Optional<ArrayList<Time>> time;
  private Optional<String> map_path;
  private Optional<String> map_name;
  private Optional<MapHeight> map_height;
  private Optional<AlternativeInput> map_overlap;
  private Optional<AlternativeInput> map_sector_width;
  private Optional<AlternativeInput> map_sector_height;
  private Optional<AlternativeInput> max_sector_users;
  private Optional<AlternativeInput> timeout;
  private Optional<AlternativeInput> gravity;
  private Optional<AlternativeInput> pedestrian_density;

  private Optional<ArrayList<Pedestrian>> pedestrians;

  private Optional<ArrayList<ExplicitVehicle>> explicit_vehicles;
  private Optional<ArrayList<RandomVehicle>> random_vehicles;
  private Optional<ArrayList<PathedVehicle>> pathed_vehicles;

  private Optional<ArrayList<Channel>> channels;

  public SimLangContainer(Scope symTab) {
    String name = symTab.getName().get() + ".";
    SimulationRenderFrequencySymbol sim_render_frequency = symTab.<SimulationRenderFrequencySymbol>resolve("simlang.test."+name+"sim_render_frequency", SimulationRenderFrequencySymbol.KIND).orElse(null);
    if(sim_render_frequency != null)
      this.sim_render_frequency = Optional.of(sim_render_frequency.getSimulationRenderFrequency());

    SimulationLoopFrequencySymbol sim_loop_frequency = symTab.<SimulationLoopFrequencySymbol>resolve(name+"sim_loop_frequency", SimulationLoopFrequencySymbol.KIND).orElse(null);
    if(sim_loop_frequency != null)
      this.sim_loop_frequency = Optional.of(sim_loop_frequency.getSimulationLoopFrequency());

    SimulationDurationSymbol sim_duration = symTab.<SimulationDurationSymbol>resolve(name+"sim_duration", SimulationDurationSymbol.KIND).orElse(null);
    if(sim_duration != null)
      this.sim_duration = Optional.of(sim_duration.getSimulationDuration());

    SimulationTypeSymbol sim_type = symTab.<SimulationTypeSymbol>resolve(name+"sim_type", SimulationTypeSymbol.KIND).orElse(null);
    if(sim_type != null)
      this.sim_type = Optional.of(sim_type.getSimulationType());

    TimeSymbol time = symTab.<TimeSymbol>resolve(name+"time", TimeSymbol.KIND).orElse(null);
    if(time != null)
      this.time = Optional.of(time.getTime());

    MapPathSymbol map_path = symTab.<MapPathSymbol>resolve(name+"map_path", MapPathSymbol.KIND).orElse(null);
    if(map_path != null)
      this.map_path = Optional.of(map_path.getMapPath());

    MapNameSymbol map_name = symTab.<MapNameSymbol>resolve(name+"map_name", MapNameSymbol.KIND).orElse(null);
    if(map_name != null)
      this.map_name = Optional.of(map_name.getMapName());

    MapHeightSymbol map_height = symTab.<MapHeightSymbol>resolve(name+"map_height", MapHeightSymbol.KIND).orElse(null);
    if(map_height != null)
      this.map_height = Optional.of(map_height.getMapHeight());

    MapOverlapSymbol map_overlap = symTab.<MapOverlapSymbol>resolve(name+"map_overlap", MapOverlapSymbol.KIND).orElse(null);
    if(sim_render_frequency != null)
      this.map_overlap = Optional.of(map_overlap.getMapOverlap());

    MapSectorWidthSymbol map_sector_width = symTab.<MapSectorWidthSymbol>resolve(name+"map_sector_width", MapSectorWidthSymbol.KIND).orElse(null);
    if(map_sector_width != null)
      this.map_sector_width = Optional.of(map_sector_width.getMapSectorWidth());

    MapSectorHeightSymbol map_sector_height = symTab.<MapSectorHeightSymbol>resolve(name+"map_sector_height", MapSectorHeightSymbol.KIND).orElse(null);
    if(map_sector_height != null)
      this.map_sector_height = Optional.of(map_sector_height.getMapSectorHeight());

    MaxSectorUsersSymbol max_sector_users = symTab.<MaxSectorUsersSymbol>resolve(name+"max_sector_users", MaxSectorUsersSymbol.KIND).orElse(null);
    if(max_sector_users != null)
      this.max_sector_users = Optional.of(max_sector_users.getMaxSectorUsers());

    TimeoutSymbol timeout = symTab.<TimeoutSymbol>resolve(name+"timeout", TimeoutSymbol.KIND).orElse(null);
    if(timeout != null)
      this.timeout = Optional.of(timeout.getTimeout());

    GravitySymbol gravity = symTab.<GravitySymbol>resolve(name+"gravity", GravitySymbol.KIND).orElse(null);
    if(gravity != null)
      this.gravity = Optional.of(gravity.getGravity());

    PedestrianDensitySymbol pedestrian_density = symTab.<PedestrianDensitySymbol>resolve(name+"pedestrian_density", PedestrianDensitySymbol.KIND).orElse(null);
    if(pedestrian_density != null)
      this.pedestrian_density = Optional.of(pedestrian_density.getPedestrianDensity());

    //DAS LÄSST SICH VEREINFACHEN DERP
    Collection<ExplicitVehicleSymbol> explicit_vehicles = symTab.<ExplicitVehicleSymbol>resolveMany(name+"explicit_vehicles", ExplicitVehicleSymbol.KIND);
    if(!explicit_vehicles.isEmpty()) {
      ArrayList<ExplicitVehicle> content = new ArrayList<>();
      for(ExplicitVehicleSymbol sym : explicit_vehicles) {
        content.add(sym.getVehicle());
      }
      this.explicit_vehicles = Optional.of(content);
    }

    Collection<PathedVehicleSymbol> pathed_vehicles = symTab.<PathedVehicleSymbol>resolveMany(name+"random_vehicles", PathedVehicleSymbol.KIND);
    if(!pathed_vehicles.isEmpty()) {
      ArrayList<PathedVehicle> content = new ArrayList<>();
      for(PathedVehicleSymbol sym : pathed_vehicles) {
        content.add(sym.getVehicle());
      }
      this.pathed_vehicles = Optional.of(content);
    }

    Collection<RandomVehicleSymbol> random_vehicles = symTab.<RandomVehicleSymbol>resolveMany(name+"random_vehicles", RandomVehicleSymbol.KIND);
    if(!random_vehicles.isEmpty()) {
      ArrayList<RandomVehicle> content = new ArrayList<>();
      for(RandomVehicleSymbol sym : random_vehicles) {
        content.add(sym.getVehicle());
      }
      this.random_vehicles = Optional.of(content);
    }

    Collection<ChannelSymbol> channels = symTab.<ChannelSymbol>resolveMany(name+"channel", ChannelSymbol.KIND);
    if(!channels.isEmpty()) {
      ArrayList<Channel> content = new ArrayList<>();
      for(ChannelSymbol sym : channels) {
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

  public Optional<ArrayList<RandomVehicle>> getRandomVehicles() {
    return this.random_vehicles;
  }

  public Optional<ArrayList<Channel>> getChannels() {
    return this.channels;
  }
}
