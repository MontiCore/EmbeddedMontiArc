/*
 * Custom CoCos for Latency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTLatency;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTLatencyCoCo;
import de.se_rwth.commons.logging.Log;

public class LatencyChecker implements SimLangASTLatencyCoCo {
  
  @Override
  public void check(ASTLatency obj) {
    System.out.println("[CoCo] LatencyChecker...");
    
    String[] allowedUnits = {"ms","s","m","h"};
    
    String input = obj.getLatency();
    
    UnitNumberChecker checker = new UnitNumberChecker(input,allowedUnits);
    
    if(!checker.inMinRange(0)) {
      Log.error("Range Error: Latency must be at least 0.");
    }
    if(!checker.legitUnit()) {
      Log.error("Unit Error: Latency missing or invalid unit.");
    }
    
    
    System.out.println("[Done] LatencyChecker");
  }
  
}
