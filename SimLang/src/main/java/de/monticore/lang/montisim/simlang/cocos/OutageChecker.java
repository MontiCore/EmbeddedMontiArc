/*
 * Custom CoCos for Timeout
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTOutage;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTOutageCoCo;
import de.monticore.lang.montisim.weather.cocos.InputHelper;
import de.monticore.lang.montisim.weather.cocos.UnitNumberChecker;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class OutageChecker implements SimLangASTOutageCoCo {
  
  @Override
  public void check(ASTOutage obj) {
    System.out.println("[CoCo] OutageChecker...");
    
    String[] allowedUnits = {""};
    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

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
