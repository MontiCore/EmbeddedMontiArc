/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package weather.cocos;

import weather._ast.ASTPressure;
import weather._cocos.WeatherASTPressureCoCo;
import de.se_rwth.commons.logging.Log;

public class PressureChecker implements WeatherASTPressureCoCo {
  
  @Override
  public void check(ASTPressure preObj) {
    System.out.println("[CoCo] PressureChecker...");
    
    String[] allowedUnits = {"Pa","kPa","mPa","hPa","bar"};
    
    String input = preObj.getWeatherPressure();
    
    UnitNumberChecker checker = new UnitNumberChecker(input,allowedUnits);
    
    if(!checker.legitUnit()) {
        Log.error("Unit Error: Pressure invalid or missing unit.");
    }
    if(checker.getUnit().contains("Pa")) {
      if(false) {
        Log.error("Range Error: Pressure in % must be between 0 and 100.");
      }
    } else {
      if(false) {
        Log.error("Range Error: Pressure as float must be between 0 and 1.");
      }
    }
    
    
    System.out.println("[Done] PressureChecker");
  }
  
}
