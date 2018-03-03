/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.weather._ast.ASTWindDirection;
import de.monticore.lang.montisim.weather._cocos.WeatherASTWindDirectionCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class WindDirectionChecker implements WeatherASTWindDirectionCoCo {
  
  @Override
  public void check(ASTWindDirection obj) {
    System.out.println("[CoCo] WindDirectionChecker...");
    
    String[] allowedUnits = {"°",""};

    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.error("Unit Error: wind_direction invalid or missing unit.");
      }
      if (checker.getDigit() < 0.0f || checker.getDigit() <= 360.0f) {
        Log.error("Range Error: wind_direction must be at least 0 and smaller 360.");
      }
    }
    
    System.out.println("[Done] WindDirectionChecker");
  }
  
}
