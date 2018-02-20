/*
 * Custom CoCos for Timeout
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTOutage;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTOutageCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class OutageChecker implements SimLangASTOutageCoCo {
  
  @Override
  public void check(ASTOutage obj) {
    System.out.println("[CoCo] OutageChecker...");
    
    String[] allowedUnits = {""};
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

      if(!checker.legitUnit()) {
        Log.error("Unit Error: Outage invalid unit.");
      }
      if(!checker.inClosedRange(0,1)) {
        Log.error("Range Error: Outage in float must be within [0,1].");
      }
    }

    System.out.println("[Done] OutageChecker");
  }
  
}
