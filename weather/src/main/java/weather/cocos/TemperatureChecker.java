/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package weather.cocos;

import weather._ast.ASTTemperature;
import weather._cocos.WeatherASTTemperatureCoCo;
import de.se_rwth.commons.logging.Log;

public class TemperatureChecker implements WeatherASTTemperatureCoCo {
  
  @Override
  public void check(ASTTemperature obj) {
    System.out.println("[CoCo] TemperatureChecker...");
    
    //Java + ° = ugh....
    String[] allowedUnits = {"K","\u00b0C","\u00b0F"};
    
    String input = obj.getWeatherTemperature();
    
    UnitNumberChecker checker = new UnitNumberChecker(input,allowedUnits);
      
    if(!checker.legitUnit()) {
        Log.error("Unit Error: Temperature invalid or missing unit.");
    }
    if(checker.getUnit().equals("K")) {
      if(!checker.inMinRange(0.0f)) {
        Log.error("Range Error: Temperature in K must be at least 0.");
      }
    } else if (checker.getUnit().equals("°C")){
      if(!checker.inMinRange(-273.15f)) {
        Log.error("Range Error: Temperature in °C must be greater than -273.15.");
      }
    } else { //assume °F
      if(!checker.inMinRange(-459.67f)) {
        Log.error("Range Error: Temperature in °F must be greater than -459.67.");
      }
    }
    
    System.out.println("[Done] TemperatureChecker");
  }
  
}
