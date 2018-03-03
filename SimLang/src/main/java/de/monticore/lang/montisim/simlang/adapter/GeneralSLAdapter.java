package de.monticore.lang.montisim.simlang.adapter;

import de.monticore.symboltable.Scope;
import de.monticore.lang.montisim.simlang.util.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

public class GeneralSLAdapter {
  private enum SimulationTypes {
    FIXED, REALTIME, MAXFPS
  }

  private Optional<AlternativeInput> sim_render_frequency = Optional.empty();
  private Optional<AlternativeInput> sim_loop_frequency;
  private Optional<AlternativeInput> sim_duration;
  private Optional<SimulationTypes> sim_type;
  private Optional<Weather> weather;
  private Optional<Time> time;
  private Optional<String> map_path;
  private Optional<String> map_name;
  private Optional<String> map_height;
  private Optional<Float> map_overlap;
  private Optional<Float> map_sector_width;
  private Optional<Float> map_sector_height;
  private Optional<Float> max_sector_users;
  private Optional<NumberUnit> timeout;
  private Optional<AlternativeInput> gravity;
  private Optional<AlternativeInput> pedestrian_density;

  private Optional<ArrayList<Pedestrian>> pedestrians;

  private Optional<ArrayList<ExplicitVehicle>> explicitVehicles;
  private Optional<ArrayList<RandomVehicle>> randomVehicles;
  private Optional<ArrayList<PathedVehicle>> pathedVehicles;

  private Optional<ArrayList<Channel>> channels;

  public GeneralSLAdapter(HashMap symMap) {
/*
    if(symMap.containsKey("sim_render_frequency")) {
      this.sim_render_frequency = Optional.of(new AlternativeInput(symMap.get("sim_render_frequency")));
    }
    if(symMap.containsKey("sim_loop_frequency")) {
      this.sim_loop_frequency = Optional.of(new AlternativeInput(symMap.get("sim_loop_frequency")));
    }
    if(symMap.containsKey("sim_duration")) {
      this.sim_duration = Optional.of(new AlternativeInput(symMap.get("sim_duration")));
    }
    if(symMap.containsKey("sim_type")) {
      switch (symMap.get("sim_type")) {
        case 0:
          this.sim_type = Optional.of(SimulationTypes.FIXED);
          break;
        case 1:
          this.sim_type = Optional.of(SimulationTypes.REALTIME);
          break;
        case 2:
          this.sim_type = Optional.of(SimulationTypes.MAXFPS);
          break;
      }
    }
    if(symMap.containsKey("sim_render_frequency")) {
      this.sim_render_frequency = Optional.of(new AlternativeInput(symMap.get("sim_render_frequency")));
    }
    if(symMap.containsKey("time")) {
      this.time = Optional.of(symMap.get("time"));
    }
    if(symMap.containsKey("map_path")) {
      this.sim_render_frequency = Optional.of(new AlternativeInput(symMap.get("sim_render_frequency")));
    }
    if(symMap.containsKey("map_name")) {
      this.sim_render_frequency = Optional.of(new AlternativeInput(symMap.get("sim_render_frequency")));
    }
    if(symMap.containsKey("map_height")) {
      this.sim_render_frequency = Optional.of(new AlternativeInput(symMap.get("sim_render_frequency")));
    }
    if(symMap.containsKey("map_overlap")) {
      this.sim_render_frequency = Optional.of(new AlternativeInput(symMap.get("sim_render_frequency")));
    }
    if(symMap.containsKey("map_sector_width")) {
      this.sim_render_frequency = Optional.of(new AlternativeInput(symMap.get("sim_render_frequency")));
    }
    if(symMap.containsKey("map_sector_height")) {
      this.sim_render_frequency = Optional.of(new AlternativeInput(symMap.get("sim_render_frequency")));
    }
    if(symMap.containsKey("max_sector_users")) {
      this.sim_render_frequency = Optional.of(new AlternativeInput(symMap.get("sim_render_frequency")));
    }
    if(symMap.containsKey("timeout")) {
      this.timeout = Optional.of(symMap.get("timeout"));
    }
    if(symMap.containsKey("gravity")) {
      this.timeout = Optional.of(symMap.get("gravity"));
    }
    if(symMap.containsKey("pedestrian_density")) {
      this.timeout = Optional.of(symMap.get("pedestrian_density"));
    }
    if(symMap.containsKey("pedestrians")) {
      this.pedestrians = Optional.of(symMap.get("pedestrians"));
    }
    if(symMap.containsKey("explicitVehicles")) {
      this.explicitVehicles = Optional.of(symMap.get("explicitVehicles"));
    }
    if(symMap.containsKey("randomVehicles")) {
      this.randomVehicles = Optional.of(symMap.get("randomVehicles"));
    }
    if(symMap.containsKey("pathedVehicles")) {
      this.pathedVehicles = Optional.of(symMap.get("pathedVehicles"));
    }
    if(symMap.containsKey("channels")) {
      this.channels = Optional.of(symMap.get("channels"));
    }*/
  }
  public GeneralSLAdapter(Scope symTab) {
    //todo
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

  public Optional<SimulationTypes> getSimulationType() {
    return this.sim_type;
  }

  public Optional<Weather> getWeather() {
    return this.weather;
  }

  public Optional<Time> getTime() {
    return this.time;
  }

  public Optional<String> getMapPath() {
    return this.map_path;
  }

  public Optional<String> getMapName() {
    return this.map_name;
  }

  public Optional<String> getMapHeight() {
    return this.map_height;
  }

  public Optional<Float> getMapOverlap() {
    return this.map_overlap;
  }

  public Optional<Float> getMapSectorWidth() {
    return this.map_sector_width;
  }

  public Optional<Float> getMapSectorHeight() {
    return this.map_sector_height;
  }

  public Optional<Float> getMaxSectorUsers() {
    return this.max_sector_users;
  }

  public Optional<NumberUnit> getTimeout() {
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
    return this.explicitVehicles;
  }

  public Optional<ArrayList<PathedVehicle>> getPathedVehicles() {
    return this.pathedVehicles;
  }

  public Optional<ArrayList<RandomVehicle>> getRandomVehicles() {
    return this.randomVehicles;
  }

  public Optional<ArrayList<Channel>> getChannels() {
    return this.channels;
  }
}
