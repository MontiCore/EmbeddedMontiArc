/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package simlang.cocos;

import simlang._ast.ASTSimulationLoopFrequency;
import si._ast.ASTUnitNumber;
import simlang._cocos.SimLangASTSimulationLoopFrequencyCoCo;
import de.se_rwth.commons.logging.Log;
import simlang.cocos.UnitNumberChecker;
import java.util.Optional;

public class SimulationLoopFrequencyChecker implements SimLangASTSimulationLoopFrequencyCoCo {
  
  @Override
  public void check(ASTSimulationLoopFrequency simLoopFreqObj) {
    System.out.println("[CoCo] SimulationLoopFrequencyChecker...");
    
    String[] allowedUnits = {"ms","s","m","h"};
    
    String input = simLoopFreqObj.getSimLoopFreq();
    
    UnitNumberChecker checker = new UnitNumberChecker(input,allowedUnits);
    
    if(!checker.inPositiveRange()) {
      Log.error("Range Error: SimulationLoopFrequency must be greater 0.");
    }
    if(!checker.legitUnit()) {
      Log.error("Unit Error: SimulationLoopFrequency missing or invalid unit.");
    }
    
    
    System.out.println("[Done] SimulationLoopFrequencyChecker");
  }
  
}
