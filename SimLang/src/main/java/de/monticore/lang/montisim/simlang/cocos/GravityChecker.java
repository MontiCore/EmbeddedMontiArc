/*
 * Custom CoCos for Gravity
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTGravity;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTGravityCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.Optional;
import java.util.ArrayList;

public class GravityChecker implements SimLangASTGravityCoCo {
  
  @Override
  public void check(ASTGravity obj) {
    System.out.println("[CoCo] GravityChecker...");
    
    String[] allowedUnits = {"m/s^2"};
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
        Log.error("Range Error: transferrate must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.error("Unit Error: transferrate missing or invalid unit.");
      }
    }


    System.out.println("[Done] GravityChecker");
  }
  
}
