/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for Latency
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTLatency;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTLatencyCoCo;
import de.monticore.lang.montisim.simlang._symboltable.LatencySymbol;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class LatencyChecker implements SimLangASTLatencyCoCo {
  
  @Override
  public void check(ASTLatency node) {
    String[] allowedUnits = {"ms","s","m","h"};
    LatencySymbol sym = (LatencySymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getLatency()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.inMinRange(0)) {
        Log.warn("Range Error: latency must be at least 0.");
      }
      if (!checker.legitUnit()) {
        Log.warn("Unit Error: latency missing or invalid unit.");
      }
    }
  }
  
}
