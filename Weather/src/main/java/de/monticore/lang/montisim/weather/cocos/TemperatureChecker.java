/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.weather._ast.ASTTemperature;
import de.monticore.lang.montisim.weather._cocos.WeatherASTTemperatureCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class TemperatureChecker implements WeatherASTTemperatureCoCo {
  
  @Override
  public void check(ASTTemperature obj) {
    //Java + ° = ugh....
    String[] allowedUnits = {"K","\u00b0C","\u00b0F"};
    ArrayList<NumberUnit> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(NumberUnit nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.error("Unit Error: Temperature invalid or missing unit.");
      }
      if (checker.getUnit().equals("K")) {
        if (!checker.inMinRange(0.0f)) {
          Log.error("Range Error: Temperature in K must be at least 0.");
        }
      } else if (checker.getUnit().equals("°C")) {
        if (!checker.inMinRange(-273.15f)) {
          Log.error("Range Error: Temperature in °C must be greater than -273.15.");
        }
      } else { //assume °F
        if (!checker.inMinRange(-459.67f)) {
          Log.error("Range Error: Temperature in °F must be greater than -459.67.");
        }
      }
    }
  }
  
}
