/*
 * Custom CoCos for SimulationDuration
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulationDuration;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationDurationCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.Optional;

public class SimulationDurationChecker implements SimLangASTSimulationDurationCoCo {
  
  @Override
  public void check(ASTSimulationDuration simDurObj) {
    System.out.println("[CoCo] SimulationDurationChecker...");
    
    String[] allowedUnits = {"ms","s","m","h"};
    
    String input = simDurObj.getSimDuration();
    
    UnitNumberChecker checker = new UnitNumberChecker(input,allowedUnits);
    
    if(!checker.inPositiveRange()) {
      Log.error("Range Error: SimulationDuration must be greater 0.");
    }
    if(!checker.legitUnit()) {
      Log.error("Unit Error: SimulationDuration missing or invalid unit.");
    }
    
    
    System.out.println("[Done] SimulationDurationChecker");
  }
  
}
