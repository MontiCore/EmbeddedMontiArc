/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.weather._ast.ASTHumidity;
import de.monticore.lang.montisim.weather._cocos.WeatherASTHumidityCoCo;
import de.se_rwth.commons.logging.Log;

public class HumidityChecker implements WeatherASTHumidityCoCo {
  
  @Override
  public void check(ASTHumidity obj) {
    System.out.println("[CoCo] HumidityChecker...");
    
    String[] allowedUnits = {""};
    
    String input = obj.getWeatherHumidity();
    
    UnitNumberChecker checker = new UnitNumberChecker(input, allowedUnits);
    
    if(!checker.legitUnit()) {
      Log.error("Unit Error: Humidity invalid unit.");
    }
    
    if(obj.isPERCENT()) {
      if(!checker.inClosedRange(0,100)) {
        Log.error("Range Error: Humidity in percent must be within [0,100].");
      }
    } else {
      if(!checker.inClosedRange(0,1)) {
        Log.error("Range Error: Humidity in float must be within [0,1].");
      }
    }
    System.out.println("[Done] HumidityChecker");
  }
  
}
