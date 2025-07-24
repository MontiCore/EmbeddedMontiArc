/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for Timeout
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTOutage;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTOutageCoCo;
import de.monticore.lang.montisim.simlang._symboltable.OutageSymbol;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class OutageChecker implements SimLangASTOutageCoCo {
  
  @Override
  public void check(ASTOutage node) {
    String[] allowedUnits = {""};
    OutageSymbol sym = (OutageSymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getOutage()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if(!checker.legitUnit()) {
        Log.warn("Unit Error: Outage invalid unit.");
      }
      if(!checker.inClosedRange(0,1)) {
        Log.warn("Range Error: Outage in float must be within [0,1].");
      }
    }
  }
  
}
