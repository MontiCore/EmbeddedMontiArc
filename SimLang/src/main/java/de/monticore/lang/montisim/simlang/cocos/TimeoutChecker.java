/*
 * Custom CoCos for Timeout
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTTimeout;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTTimeoutCoCo;
import de.monticore.lang.montisim.weather.cocos.InputHelper;
import de.monticore.lang.montisim.weather.cocos.UnitNumberChecker;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Optional;

public class TimeoutChecker implements SimLangASTTimeoutCoCo {
  
  @Override
  public void check(ASTTimeout obj) {
    System.out.println("[CoCo] TimeoutChecker...");
    
    String[] allowedUnits = {"ms","s","m","h"};
    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.error("Range Error: timeout must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.error("Unit Error: timeout missing or invalid unit.");
      }
    }
    System.out.println("[Done] TimeoutChecker");
  }
  
}
