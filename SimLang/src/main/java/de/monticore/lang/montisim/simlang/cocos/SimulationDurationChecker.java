/*
 * Custom CoCos for SimulationDuration
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulationDuration;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationDurationCoCo;
import de.monticore.lang.montisim.weather.cocos.InputHelper;
import de.monticore.lang.montisim.weather.cocos.UnitNumberChecker;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class SimulationDurationChecker implements SimLangASTSimulationDurationCoCo {
  
  @Override
  public void check(ASTSimulationDuration obj) {
    String[] allowedUnits = {"ms","s","m","h"};
    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.error("Range Error: sim_duration must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.error("Unit Error: sim_duration missing or invalid unit.");
      }
    }
  }
  
}
