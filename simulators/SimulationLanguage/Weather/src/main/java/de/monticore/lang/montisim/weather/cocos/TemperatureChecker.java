/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for SimulationLoopFrequency
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.weather._ast.ASTTemperature;
import de.monticore.lang.montisim.weather._cocos.WeatherASTTemperatureCoCo;
import de.monticore.lang.montisim.weather.symboltable.TemperatureSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class TemperatureChecker implements WeatherASTTemperatureCoCo {
  
  @Override
  public void check(ASTTemperature node) {
    //Java + ° = ugh....
    String[] allowedUnits = {"K","\u00b0C","\u00b0F"};
    TemperatureSymbol sym = (TemperatureSymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getTemperature()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.warn("Unit Error: Temperature invalid or missing unit.");
      }
      if (checker.getUnit().equals("K")) {
        if (!checker.inMinRange(0.0f)) {
          Log.warn("Range Error: Temperature in K must be at least 0.");
        }
      } else if (checker.getUnit().equals("°C")) {
        if (!checker.inMinRange(-273.15f)) {
          Log.warn("Range Error: Temperature in °C must be greater than -273.15.");
        }
      } else { //assume °F
        if (!checker.inMinRange(-459.67f)) {
          Log.warn("Range Error: Temperature in °F must be greater than -459.67.");
        }
      }
    }
  }
  
}
