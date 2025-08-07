/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for SimulationLoopFrequency
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.weather._ast.ASTHumidity;
import de.monticore.lang.montisim.weather._cocos.WeatherASTHumidityCoCo;
import de.monticore.lang.montisim.weather.symboltable.HumiditySymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class HumidityChecker implements WeatherASTHumidityCoCo {
  
  @Override
  public void check(ASTHumidity node) {
    String[] allowedUnits = {""};
    HumiditySymbol sym = (HumiditySymbol)node.getSymbol().get();
    
    ArrayList<NumberUnit> input = new InputHelper(sym.getHumidity()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.warn("Unit Error: Humidity invalid unit.");
      }
      if (!checker.inClosedRange(0, 1)) {
        Log.warn("Range Error: Humidity in float must be within [0,1].");
      }
    }
  }
  
}
