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
    List<de.monticore.lang.montisim.simlang._ast.ASTSimulationRenderFrequency> simRenderFreq = sim.getSimulationRenderFrequencys();
    List<de.monticore.lang.montisim.simlang._ast.ASTSimulationLoopFrequency> simLoopFreq = sim.getSimulationLoopFrequencys();
    List<de.monticore.lang.montisim.simlang._ast.ASTSimulationDuration> simDuration = sim.getSimulationDurations();
    List<de.monticore.lang.montisim.simlang._ast.ASTSimulationType> simType = sim.getSimulationTypes();
    List<de.monticore.lang.montisim.simlang._ast.ASTWeather> weather = sim.getWeathers();
    List<de.monticore.lang.montisim.simlang._ast.ASTTime> time = sim.getTimes();
    List<de.monticore.lang.montisim.simlang._ast.ASTMapPath> mapPath = sim.getMapPaths();
    List<de.monticore.lang.montisim.simlang._ast.ASTMapName> mapName = sim.getMapNames();
    List<de.monticore.lang.montisim.simlang._ast.ASTMapHeight> mapHeight = sim.getMapHeights();
    List<de.monticore.lang.montisim.simlang._ast.ASTMapOverlap> mapOverlap = sim.getMapOverlaps();
    List<de.monticore.lang.montisim.simlang._ast.ASTMapSectorWidth> mapSectorWidth = sim.getMapSectorWidths();
    List<de.monticore.lang.montisim.simlang._ast.ASTMapSectorHeight> mapSectorHeight = sim.getMapSectorHeights();
    List<de.monticore.lang.montisim.simlang._ast.ASTMaxSectorUsers> maxSectorUser = sim.getMaxSectorUserss();
    List<de.monticore.lang.montisim.simlang._ast.ASTTimeout> timeout = sim.getTimeouts();
    List<de.monticore.lang.montisim.simlang._ast.ASTPedestrianDensity> pedestrianDensity = sim.getPedestrianDensitys();
    
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
  }
  
}
