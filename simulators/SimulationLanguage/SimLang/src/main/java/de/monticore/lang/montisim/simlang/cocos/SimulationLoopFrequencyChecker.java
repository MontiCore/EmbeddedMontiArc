/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for SimulationLoopFrequency
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulationLoopFrequency;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationLoopFrequencyCoCo;
import de.monticore.lang.montisim.simlang._symboltable.SimulationLoopFrequencySymbol;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class SimulationLoopFrequencyChecker implements SimLangASTSimulationLoopFrequencyCoCo {
  
  @Override
  public void check(ASTSimulationLoopFrequency node) {
    String[] allowedUnits = {"ms","s","m","h"};
    SimulationLoopFrequencySymbol sym = (SimulationLoopFrequencySymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getSimulationLoopFrequency()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.warn("Range Error: sim_loop_frequency must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.warn("Unit Error: sim_loop_frequency missing or invalid unit.");
      }
    }
  }
  
}
