/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for SimulationLoopFrequency
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.weather._ast.ASTWindDirection;
import de.monticore.lang.montisim.weather._cocos.WeatherASTWindDirectionCoCo;
import de.monticore.lang.montisim.weather.symboltable.WindDirectionSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;


public class WindDirectionChecker implements WeatherASTWindDirectionCoCo {
  
  @Override
  public void check(ASTWindDirection node) {
    String[] allowedUnits = {"°",""};
    WindDirectionSymbol sym = (WindDirectionSymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getWindDirection()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.warn("Unit Error: wind_direction invalid or missing unit.");
      }
      if (checker.getDigit() < 0.0f || checker.getDigit() <= 360.0f) {
        Log.warn("Range Error: wind_direction must be at least 0 and smaller 360.");
      }
    }
  }
  
}
