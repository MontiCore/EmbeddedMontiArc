/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.weather._ast.ASTWindStrength;
import de.monticore.lang.montisim.weather._cocos.WeatherASTWindStrengthCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class WindStrengthChecker implements WeatherASTWindStrengthCoCo {
  
  @Override
  public void check(ASTWindStrength obj) {
    System.out.println("[CoCo] WindStrengthChecker...");
    
    String[] allowedUnits = {"knots","m/s","km/h","mph"};

    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.error("Unit Error: wind_strength invalid or missing unit.");
      }
      if (!checker.inMinRange(0)) {
        Log.error("Range Error: wind_strength must be at least 0.");
      }
    }
    
    System.out.println("[Done] WindStrengthChecker");
  }
  
}
