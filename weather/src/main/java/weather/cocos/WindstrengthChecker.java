/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package weather.cocos;

import weather._ast.ASTWindstrength;
import weather._cocos.WeatherASTWindstrengthCoCo;
import de.se_rwth.commons.logging.Log;

public class WindstrengthChecker implements WeatherASTWindstrengthCoCo {
  
  @Override
  public void check(ASTWindstrength obj) {
    System.out.println("[CoCo] WindstrengthChecker...");
    
    String[] allowedUnits = {"knots","m/s","km/h","mph"};
    
    String input = obj.getWeatherWindstrength();
    
    UnitNumberChecker checker = new UnitNumberChecker(input,allowedUnits);
      
    if(!checker.legitUnit()) {
        Log.error("Unit Error: Windstrength invalid or missing unit.");
    }
    if(!checker.inMinRange(0)) {
      Log.error("Range Error: Windstrength must be at least 0.");
    }
    
    if(checker.getUnit().equals("knots")) {
      if(false) {
        Log.error("Range Error: Windstrength in % must be between 0 and 100.");
      }
    } else if(checker.getUnit().equals("m/s")){
      if(false) {
        Log.error("Range Error: Windstrength as float must be between 0 and 1.");
      }
    } else if(checker.getUnit().equals("km/h")){
      if(false) {
        Log.error("Range Error: Windstrength as float must be between 0 and 1.");
      }
    } else if(checker.getUnit().equals("mph")){
      if(false) {
        Log.error("Range Error: Windstrength as float must be between 0 and 1.");
      }
    }
    
    
    System.out.println("[Done] WindstrengthChecker");
  }
  
}
