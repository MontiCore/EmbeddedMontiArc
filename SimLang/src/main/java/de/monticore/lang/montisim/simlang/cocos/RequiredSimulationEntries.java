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

public class RequiredSimulationEntries implements SimLangASTSimulationCoCo {
  
  @Override
  public void check(ASTSimulation sim) {
    List<de.monticore.lang.montisim.simlang._ast.ASTMapName> mapName = sim.getMapNames();
    
    if(mapName.size() < 1) {
      Log.error("Semantic Error: Map name attribute missing.");
    }
  }
  
}
