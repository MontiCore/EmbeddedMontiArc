/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSimulationLoopFrequency;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSimulationLoopFrequencyCoCo;
import de.monticore.lang.montisim.weather.cocos.InputHelper;
import de.monticore.lang.montisim.weather.cocos.UnitNumberChecker;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class SimulationLoopFrequencyChecker implements SimLangASTSimulationLoopFrequencyCoCo {
  
  @Override
  public void check(ASTSimulationLoopFrequency obj) {
    System.out.println("[CoCo] SimulationLoopFrequencyChecker...");
    
    String[] allowedUnits = {"ms","s","m","h"};
    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.error("Range Error: sim_loop_frequency must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.error("Unit Error: sim_loop_frequency missing or invalid unit.");
      }
    }
    System.out.println("[Done] SimulationLoopFrequencyChecker");
  }
  
}
