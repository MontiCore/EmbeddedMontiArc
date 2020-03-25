/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for SimulationRenderFrequency
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulationRenderFrequency;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationRenderFrequencyCoCo;
import de.monticore.lang.montisim.simlang._symboltable.SimulationRenderFrequencySymbol;
import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class SimulationRenderFrequencyChecker implements SimLangASTSimulationRenderFrequencyCoCo {
  
  @Override
  public void check(ASTSimulationRenderFrequency node) {
    String[] allowedUnits = {"ms","s","m","h"};
    SimulationRenderFrequencySymbol sym = (SimulationRenderFrequencySymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getSimulationRenderFrequency()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.warn("Range Error: sim_render_frequency must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.warn("Unit Error: sim_render_frequency missing or invalid unit.");
      }
    }
  }
  
}
