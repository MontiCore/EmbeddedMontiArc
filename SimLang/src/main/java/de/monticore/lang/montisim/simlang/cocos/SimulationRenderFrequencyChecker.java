/*
 * Custom CoCos for SimulationRenderFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulationRenderFrequency;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationRenderFrequencyCoCo;
import de.monticore.lang.montisim.weather.cocos.InputHelper;
import de.monticore.lang.montisim.weather.cocos.NumberUnit;
import de.monticore.lang.montisim.weather.cocos.UnitNumberChecker;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class SimulationRenderFrequencyChecker implements SimLangASTSimulationRenderFrequencyCoCo {
  
  @Override
  public void check(ASTSimulationRenderFrequency obj) {
    String[] allowedUnits = {"ms","s","m","h"};
    ArrayList<NumberUnit> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(NumberUnit nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.error("Range Error: sim_render_frequency must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.error("Unit Error: sim_render_frequency missing or invalid unit.");
      }
    }
  }
  
}
