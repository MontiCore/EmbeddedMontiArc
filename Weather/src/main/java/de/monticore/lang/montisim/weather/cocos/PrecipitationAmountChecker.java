/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.weather._ast.ASTPrecipitationAmount;
import de.monticore.lang.montisim.weather._cocos.WeatherASTPrecipitationAmountCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class PrecipitationAmountChecker implements WeatherASTPrecipitationAmountCoCo {
  
  @Override
  public void check(ASTPrecipitationAmount obj) {
    String[] allowedUnits = {"l/m^2", "mm"};

    ArrayList<NumberUnit> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(NumberUnit nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.error("Unit Error: precipitation_amount invalid or missing unit.");
      }
      if (!checker.inMinRange(0)) {
        Log.error("Range Error: precipitation_amount must be at least 0.");
      }
    }
  }
  
}
