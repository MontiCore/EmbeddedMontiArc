/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for Gravity
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTGravity;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTGravityCoCo;
import de.monticore.lang.montisim.simlang._symboltable.GravitySymbol;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.se_rwth.commons.logging.Log;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.cocos.InputHelper;
import java.util.ArrayList;

public class GravityChecker implements SimLangASTGravityCoCo {
  
  @Override
  public void check(ASTGravity node) {
    String[] allowedUnits = {"m/s^2","m/s²","g"};
    GravitySymbol sym = (GravitySymbol)node.getSymbol().get();
    ArrayList<NumberUnit> input = new InputHelper(sym.getGravity()).getExtractedValues();

    for (NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.inMinRange(0)) {
        Log.warn("Range Error: gravity must be at least 0.");
      }
      if (!checker.legitUnit()) {
        Log.warn("Unit Error: gravity missing or invalid unit." + "Got: "+checker.getUnit());
      }
    }
  }
  
}
