/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package weather.cocos;

import weather._ast.ASTPrecipitationamount;
import weather._cocos.WeatherASTPrecipitationamountCoCo;
import de.se_rwth.commons.logging.Log;

public class PrecipitationamountChecker implements WeatherASTPrecipitationamountCoCo {
  
  @Override
  public void check(ASTPrecipitationamount obj) {
    System.out.println("[CoCo] PrecipitationamountChecker...");
    
    String[] allowedUnits = {"l/m^2","mm"};
    
    String input = obj.getWeatherPrecipitationamount();
    
    UnitNumberChecker checker = new UnitNumberChecker(input,allowedUnits);
    
    if(!checker.legitUnit()) {
        Log.error("Unit Error: Precipitationamount invalid or missing unit.");
    }
    if(!checker.inMinRange(0)) {
        Log.error("Range Error: Precipitationamount must be at least 0.");
    }
    
    System.out.println("[Done] PrecipitationamountChecker");
  }
  
}
