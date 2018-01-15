/*
 * Custom CoCos for SimulationRenderFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package simlang.cocos;

import simlang._ast.ASTSimulationRenderFrequency;
import si._ast.ASTUnitNumber;
import simlang._cocos.SimLangASTSimulationRenderFrequencyCoCo;
import de.se_rwth.commons.logging.Log;
import simlang.cocos.UnitNumberChecker;
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
