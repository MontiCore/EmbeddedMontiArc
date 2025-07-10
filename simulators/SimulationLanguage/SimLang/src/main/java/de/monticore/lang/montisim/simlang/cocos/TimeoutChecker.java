/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for Timeout
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTTimeout;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTTimeoutCoCo;
import de.monticore.lang.montisim.simlang._symboltable.TimeoutSymbol;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class TimeoutChecker implements SimLangASTTimeoutCoCo {
  
  @Override
  public void check(ASTTimeout node) {
    String[] allowedUnits = {"ms","s","m","h"};
    TimeoutSymbol sym = (TimeoutSymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getTimeout()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.warn("Range Error: timeout must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.warn("Unit Error: timeout missing or invalid unit.");
      }
    }
  }
  
}
