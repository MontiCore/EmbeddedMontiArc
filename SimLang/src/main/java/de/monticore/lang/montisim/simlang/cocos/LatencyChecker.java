/*
 * Custom CoCos for Latency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTLatency;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTLatencyCoCo;
import de.monticore.lang.montisim.weather.cocos.InputHelper;
import de.monticore.lang.montisim.weather.cocos.UnitNumberChecker;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class LatencyChecker implements SimLangASTLatencyCoCo {
  
  @Override
  public void check(ASTLatency obj) {
    System.out.println("[CoCo] LatencyChecker...");
    
    String[] allowedUnits = {"ms","s","m","h"};
    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.inMinRange(0)) {
        Log.error("Range Error: latency must be at least 0.");
      }
      if (!checker.legitUnit()) {
        Log.error("Unit Error: latency missing or invalid unit.");
      }
    }
    
    System.out.println("[Done] LatencyChecker");
  }
  
}
