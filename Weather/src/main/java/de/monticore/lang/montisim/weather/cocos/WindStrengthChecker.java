/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for SimulationLoopFrequency
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.weather._ast.ASTWindStrength;
import de.monticore.lang.montisim.weather._cocos.WeatherASTWindStrengthCoCo;
import de.monticore.lang.montisim.weather.symboltable.WindStrengthSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class WindStrengthChecker implements WeatherASTWindStrengthCoCo {
  
  @Override
  public void check(ASTWindStrength node) {
    String[] allowedUnits = {"knots","m/s","km/h","mph"};
    WindStrengthSymbol sym = (WindStrengthSymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getWindStrength()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.warn("Unit Error: wind_strength invalid or missing unit.");
      }
      if (!checker.inMinRange(0)) {
        Log.warn("Range Error: wind_strength must be at least 0.");
      }
    }
  }
  
}
