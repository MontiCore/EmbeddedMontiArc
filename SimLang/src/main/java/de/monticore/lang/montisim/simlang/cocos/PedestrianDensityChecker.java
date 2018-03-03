/*
 * Custom CoCos for PedestrianDensity
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTPedestrianDensity;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTPedestrianDensityCoCo;
import de.monticore.lang.montisim.weather.cocos.InputHelper;
import de.monticore.lang.montisim.weather.cocos.UnitNumberChecker;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Optional;

public class PedestrianDensityChecker implements SimLangASTPedestrianDensityCoCo {
  
  @Override
  public void check(ASTPedestrianDensity obj) {
    System.out.println("[CoCo] PedestrianDensityChecker...");

    String[] allowedUnits = {""};
    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.error("Range Error: pedestrian_density must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.error("Unit Error: pedestrian_density missing or invalid unit.");
      }
    }
    System.out.println("[Done] PedestrianDensityChecker");
  }
  
}
