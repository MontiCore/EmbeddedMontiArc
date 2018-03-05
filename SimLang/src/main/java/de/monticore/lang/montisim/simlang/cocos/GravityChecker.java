/*
 * Custom CoCos for Gravity
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTGravity;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTGravityCoCo;
import de.se_rwth.commons.logging.Log;
import de.monticore.lang.montisim.weather.cocos.UnitNumberChecker;
import de.monticore.lang.montisim.weather.cocos.InputHelper;
import java.util.ArrayList;

public class GravityChecker implements SimLangASTGravityCoCo {
  
  @Override
  public void check(ASTGravity obj) {
    String[] allowedUnits = {"m/s^2"};
    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for (String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.inMinRange(0)) {
        Log.error("Range Error: gravity must be at least 0.");
      }
      if (!checker.legitUnit()) {
        Log.error("Unit Error: gravity missing or invalid unit.");
      }
    }
  }
  
}
