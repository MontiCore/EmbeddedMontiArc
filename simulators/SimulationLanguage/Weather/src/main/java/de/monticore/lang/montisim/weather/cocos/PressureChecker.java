/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for SimulationLoopFrequency
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.weather._ast.ASTPressure;
import de.monticore.lang.montisim.weather._cocos.WeatherASTPressureCoCo;
import de.monticore.lang.montisim.weather.symboltable.PressureSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class PressureChecker implements WeatherASTPressureCoCo {
  
  @Override
  public void check(ASTPressure node) {
    String[] allowedUnits = {"Pa","kPa","mPa","hPa","bar"};
    PressureSymbol sym = (PressureSymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getPressure()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.warn("Unit Error: Pressure invalid or missing unit.");
      }
      if (!checker.inMinRange(0)) {
        Log.warn("Range Error: Pressure  must be at least 0.");
      }
    }
  }
  
}
