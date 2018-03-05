/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.weather._ast.ASTHumidity;
import de.monticore.lang.montisim.weather._cocos.WeatherASTHumidityCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class HumidityChecker implements WeatherASTHumidityCoCo {
  
  @Override
  public void check(ASTHumidity obj) {
    String[] allowedUnits = {""};
    
    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.error("Unit Error: Humidity invalid unit.");
      }
      if (!checker.inClosedRange(0, 1)) {
        Log.error("Range Error: Humidity in float must be within [0,1].");
      }
    }
  }
  
}
