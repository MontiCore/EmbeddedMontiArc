/*
 * Custom CoCos for Timeout
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTTransferrate;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTTransferrateCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.Optional;

public class TransferrateChecker implements SimLangASTTransferrateCoCo {
  
  @Override
  public void check(ASTTransferrate obj) {
    System.out.println("[CoCo] TransferrateChecker...");
    
    String[] allowedUnits = {"bit/s","kbit/s","Mbit/s","Gbit/s"};
    
    String input = obj.getTransferrate();
    
    UnitNumberChecker checker = new UnitNumberChecker(input,allowedUnits);
    
    if(!checker.inPositiveRange()) {
      Log.error("Range Error: Transferrate must be greater 0.");
    }
    if(!checker.legitUnit()) {
      Log.error("Unit Error: Transferrate missing or invalid unit.");
    }
    
    
    System.out.println("[Done] TransferrateChecker");
  }
  
}
