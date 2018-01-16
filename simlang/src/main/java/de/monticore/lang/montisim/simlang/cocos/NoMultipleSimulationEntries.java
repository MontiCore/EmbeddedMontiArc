/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulation;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.List;

public class NoMultipleSimulationEntries implements SimLangASTSimulationCoCo {
  
  @Override
  public void check(ASTSimulation sim) {
    System.out.println("[CoCo] NoMultipleSimlulationEntries...");
    List<simlang._ast.ASTSimulationRenderFrequency> simRenderFreq = sim.getSimulationRenderFrequencys();
    List<simlang._ast.ASTSimulationLoopFrequency> simLoopFreq = sim.getSimulationLoopFrequencys();
    List<simlang._ast.ASTSimulationDuration> simDuration = sim.getSimulationDurations();
    List<simlang._ast.ASTSimulationType> simType = sim.getSimulationTypes();
    List<simlang._ast.ASTWeather> weather = sim.getWeathers();
    List<simlang._ast.ASTTime> time = sim.getTimes();
    List<simlang._ast.ASTMapPath> mapPath = sim.getMapPaths();
    List<simlang._ast.ASTMapName> mapName = sim.getMapNames();
    List<simlang._ast.ASTMapHeight> mapHeight = sim.getMapHeights();
    List<simlang._ast.ASTMapOverlap> mapOverlap = sim.getMapOverlaps();
    List<simlang._ast.ASTMapSectorWidth> mapSectorWidth = sim.getMapSectorWidths();
    List<simlang._ast.ASTMapSectorHeight> mapSectorHeight = sim.getMapSectorHeights();
    List<simlang._ast.ASTMaxSectorUsers> maxSectorUser = sim.getMaxSectorUserss();
    List<simlang._ast.ASTTimeout> timeout = sim.getTimeouts();
    List<simlang._ast.ASTPedestrianDensity> pedestrianDensity = sim.getPedestrianDensitys();
    
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
       pedestrianDensity.size() > 1
    ) {
      Log.error("Semantic Error: A simulation attribute was defined more than once.");
    }
    
    System.out.println("[Done] NoMultipleSimulationEntries");
  }
  
}
