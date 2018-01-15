/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package weather.cocos;

import weather._ast.ASTWinddirection;
import weather._cocos.WeatherASTWinddirectionCoCo;
import de.se_rwth.commons.logging.Log;

public class WinddirectionChecker implements WeatherASTWinddirectionCoCo {
  
  @Override
  public void check(ASTWinddirection obj) {
    System.out.println("[CoCo] WinddirectionChecker...");
    
    String[] allowedUnits = {"°"};
    
    String input = obj.getWeatherWinddirection();
    
    UnitNumberChecker checker = new UnitNumberChecker(input,allowedUnits);
    
    if(!checker.legitUnit()) {
        Log.error("Unit Error: Winddirection invalid or missing unit.");
    }
    if(checker.getDigit() < 0.0f || checker.getDigit() <= 360.0f) {
      Log.error("Range Error: Winddirection must be at least 0 and smaller 360.");
    }
    
    
    System.out.println("[Done] WinddirectionChecker");
  }
  
}
