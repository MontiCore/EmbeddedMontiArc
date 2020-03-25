/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for SimulationDuration
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulationDuration;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationDurationCoCo;
import de.monticore.lang.montisim.simlang._symboltable.SimulationDurationSymbol;
import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class SimulationDurationChecker implements SimLangASTSimulationDurationCoCo {
  
  @Override
  public void check(ASTSimulationDuration node) {
    String[] allowedUnits = {"ms","s","m","h"};
    SimulationDurationSymbol sym = (SimulationDurationSymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getSimulationDuration()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.warn("Range Error: sim_duration must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.warn("Unit Error: sim_duration missing or invalid unit.");
      }
    }
  }
  
}
