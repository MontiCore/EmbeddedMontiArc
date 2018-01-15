/*
 * Custom CoCos for Timeout
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package simlang.cocos;

import simlang._ast.ASTOutage;
import simlang._cocos.SimLangASTOutageCoCo;
import de.se_rwth.commons.logging.Log;
import simlang.cocos.UnitNumberChecker;

public class OutageChecker implements SimLangASTOutageCoCo {
  
  @Override
  public void check(ASTOutage obj) {
    System.out.println("[CoCo] OutageChecker...");
    
    String[] allowedUnits = {""};
    String input = obj.getOutage();
    
    UnitNumberChecker checker = new UnitNumberChecker(input, allowedUnits);
    
    if(!checker.legitUnit()) {
        Log.error("Unit Error: Outage invalid or missing unit.");
    }
    
    if(obj.isPERCENT()) {
      if(!checker.inClosedRange(0,100)) {
        Log.error("Range Error: Outage in percent must be within [0,100].");
      }
    } else {
      if(!checker.inClosedRange(0,1)) {
        Log.error("Range Error: Outage in float must be within [0,1].");
      }
    }
    System.out.println("[Done] OutageChecker");
  }
  
}
