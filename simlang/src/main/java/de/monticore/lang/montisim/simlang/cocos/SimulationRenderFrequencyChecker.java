/*
 * Custom CoCos for SimulationRenderFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulationRenderFrequency;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationRenderFrequencyCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.Optional;

public class SimulationRenderFrequencyChecker implements SimLangASTSimulationRenderFrequencyCoCo {
  
  @Override
  public void check(ASTSimulationRenderFrequency obj) {
    System.out.println("[CoCo] SimulationRenderFrequencyChecker...");
    
    String[] allowedUnits = {"ms","s","m","h"};
    
    String input = obj.getSimRenderFreq();
    
    UnitNumberChecker checker = new UnitNumberChecker(input,allowedUnits);
    
    if(!checker.inPositiveRange()) {
      Log.error("Range Error: SimulationRenderFrequency must be greater 0.");
    }
    if(!checker.legitUnit()) {
      Log.error("Unit Error: SimulationRenderFrequency missing or invalid unit.");
    }
    
    
    System.out.println("[Done] SimulationRenderFrequencyChecker");
  }
  
}
