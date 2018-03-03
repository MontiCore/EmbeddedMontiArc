/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.weather._ast.ASTPressure;
import de.monticore.lang.montisim.weather._cocos.WeatherASTPressureCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class PressureChecker implements WeatherASTPressureCoCo {
  
  @Override
  public void check(ASTPressure obj) {
    System.out.println("[CoCo] PressureChecker...");
    String[] allowedUnits = {"Pa","kPa","mPa","hPa","bar"};
    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.error("Unit Error: Pressure invalid or missing unit.");
      }
      if (!checker.inMinRange(0)) {
        Log.error("Range Error: Pressure  must be at least 0.");
      }
    }
    
    System.out.println("[Done] PressureChecker");
  }
  
}
