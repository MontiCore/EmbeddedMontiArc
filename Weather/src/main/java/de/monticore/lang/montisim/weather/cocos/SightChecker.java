/*
 * Custom CoCos for SimulationLoopFrequency
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.weather._ast.ASTSight;
import de.monticore.lang.montisim.weather._cocos.WeatherASTSightCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class SightChecker implements WeatherASTSightCoCo {
  
  @Override
  public void check(ASTSight obj) {
    String[] allowedUnits = {"mm","cm","dm","m","km"};
    
    if(obj.isUnlimited()) {
      return;
    }
    ArrayList<NumberUnit> input = new InputHelper(obj.getAlternativeInput().get()).getExtractedValues();

    for(NumberUnit nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.error("Unit Error: Sight invalid or missing unit.");
      }
      if (!checker.inMinRange(0)) {
        Log.error("Range Error: Sight must be at least 0.");
      }
    }
  }
  
}
