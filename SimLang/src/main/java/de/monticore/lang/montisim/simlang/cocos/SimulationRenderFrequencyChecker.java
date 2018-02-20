/*
 * Custom CoCos for SimulationRenderFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulationRenderFrequency;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationRenderFrequencyCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class SimulationRenderFrequencyChecker implements SimLangASTSimulationRenderFrequencyCoCo {
  
  @Override
  public void check(ASTSimulationRenderFrequency obj) {
    System.out.println("[CoCo] SimulationRenderFrequencyChecker...");
    
    String[] allowedUnits = {"ms","s","m","h"};
    ArrayList<String> toCheck;

    if(obj.getTUnitNumber().isPresent()) {
      toCheck = new InputHelper(obj.getTUnitNumber()).getExtractedValues();
    }
    else if(obj.getTUnitNumberList().isPresent()) {
      toCheck = new InputHelper(obj.getTUnitNumberList()).getExtractedValues();
    }
    else if(obj.getRange().isPresent()) {
      toCheck = new InputHelper(obj.getRange()).getExtractedValues();
    }
    else if(obj.getLambda().isPresent()) {
      toCheck = new InputHelper(obj.getLambda()).getExtractedValues();
    }
    else {
      Log.error("Unexpected code region entered in CoCo.");
      toCheck = new ArrayList<>(); //unreachable, but IDE wants it
    }

    for(String num : toCheck) {
      UnitNumberChecker checker = new UnitNumberChecker(num, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.error("Range Error: SimulationRenderFrequency must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.error("Unit Error: SimulationRenderFrequency missing or invalid unit.");
      }
    }
    
    System.out.println("[Done] SimulationRenderFrequencyChecker");
  }
  
}
