/*
 * Custom CoCos for Timeout
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTTimeout;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTTimeoutCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.Optional;

public class TimeoutChecker implements SimLangASTTimeoutCoCo {
  
  @Override
  public void check(ASTTimeout obj) {
    System.out.println("[CoCo] TimeoutChecker...");
    
    String[] allowedUnits = {"ms","s","m","h"};
    
    String input = obj.getTimeout();
    
    UnitNumberChecker checker = new UnitNumberChecker(input,allowedUnits);
    
    if(!checker.inPositiveRange()) {
      Log.error("Range Error: Timeout must be greater 0.");
    }
    if(!checker.legitUnit()) {
      Log.error("Unit Error: Timeout missing or invalid unit.");
    }
    
    
    System.out.println("[Done] TimeoutChecker");
  }
  
}
