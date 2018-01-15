/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package simlang.cocos;

import simlang._ast.ASTSimulation;
import simlang._cocos.SimLangASTSimulationCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.List;

public class RequiredSimulationEntries implements SimLangASTSimulationCoCo {
  
  @Override
  public void check(ASTSimulation sim) {
    System.out.println("[CoCo] RequiredsimlangEntries...");
    List<simlang._ast.ASTMapName> mapName = sim.getMapNames();
    
    if(mapName.size() < 1) {
      Log.error("Semantic Error: Map name attribute missing.");
    }
    
    System.out.println("[Done] RequiredsimlangEntries");
  }
  
}
