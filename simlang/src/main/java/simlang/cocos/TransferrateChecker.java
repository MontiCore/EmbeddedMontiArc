/*
 * Custom CoCos for Timeout
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package simlang.cocos;

import simlang._ast.ASTTransferrate;
import simlang._cocos.SimLangASTTransferrateCoCo;
import de.se_rwth.commons.logging.Log;
import simlang.cocos.UnitNumberChecker;
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
