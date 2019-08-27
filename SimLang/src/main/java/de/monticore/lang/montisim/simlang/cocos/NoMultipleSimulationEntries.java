/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulation;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationCoCo;
import de.monticore.lang.montisim.simlang._symboltable.*;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;

public class NoMultipleSimulationEntries implements SimLangASTSimulationCoCo {
  
  @Override
  public void check(ASTSimulation sim) {
    Collection<SimulationRenderFrequencySymbol> simRenderFreq = sim.getEnclosingScope().get().resolveMany("sim_render_frequency",SimulationRenderFrequencySymbol.KIND);
    Collection<SimulationLoopFrequencySymbol> simLoopFreq = sim.getEnclosingScope().get().resolveMany("sim_loop_frequency", SimulationLoopFrequencySymbol.KIND);
    Collection<SimulationDurationSymbol> simDuration = sim.getEnclosingScope().get().resolveMany("sim_duration",SimulationDurationSymbol.KIND);
    Collection<SimulationTypeSymbol> simType = sim.getEnclosingScope().get().resolveMany("sim_type",SimulationTypeSymbol.KIND);
    Collection<WeatherSymbol> weather = sim.getEnclosingScope().get().resolveMany("weather",WeatherSymbol.KIND);
    Collection<TimeSymbol> time = sim.getEnclosingScope().get().resolveMany("time",TimeSymbol.KIND);
    Collection<MapPathSymbol> mapPath = sim.getEnclosingScope().get().resolveMany("map_path",MapPathSymbol.KIND);
    Collection<MapNameSymbol> mapName = sim.getEnclosingScope().get().resolveMany("map_name",MapNameSymbol.KIND);
    Collection<MapHeightSymbol> mapHeight = sim.getEnclosingScope().get().resolveMany("map_height",MapHeightSymbol.KIND);
    Collection<MapOverlapSymbol> mapOverlap = sim.getEnclosingScope().get().resolveMany("map_overlap",MapOverlapSymbol.KIND);
    Collection<MapSectorWidthSymbol> mapSectorWidth = sim.getEnclosingScope().get().resolveMany("map_sector_width",MapSectorWidthSymbol.KIND);
    Collection<MapSectorHeightSymbol> mapSectorHeight = sim.getEnclosingScope().get().resolveMany("map_sector_height",MapSectorHeightSymbol.KIND);
    Collection<MaxSectorUsersSymbol> maxSectorUser = sim.getEnclosingScope().get().resolveMany("max_sector_users",MaxSectorUsersSymbol.KIND);
    Collection<TimeoutSymbol> timeout = sim.getEnclosingScope().get().resolveMany("timeout",TimeoutSymbol.KIND);
    Collection<PedestrianDensitySymbol> pedestrianDensity = sim.getEnclosingScope().get().resolveMany("pedestrian_density",PedestrianDensitySymbol.KIND);
    Collection<GravitySymbol> gravity = sim.getEnclosingScope().get().resolveMany("gravity",GravitySymbol.KIND);
    
    if(simRenderFreq.size() > 1 |
       simLoopFreq.size() > 1 |
       simDuration.size() > 1 |
       simType.size() > 1 |
       weather.size() > 1 |
       time.size() > 1 |
       mapPath.size() > 1 |
       mapName.size() > 1 |
       mapHeight.size() > 1 |
       mapOverlap.size() > 1 |
       mapSectorWidth.size() > 1 |
       mapSectorHeight.size() > 1 |
       maxSectorUser.size() > 1 |
       timeout.size() > 1 |
       pedestrianDensity.size() > 1 |
            gravity.size() > 1
    ) {
      Log.warn("Semantic Error: A simulation attribute was defined more than once.");
    }
  }
  
}
