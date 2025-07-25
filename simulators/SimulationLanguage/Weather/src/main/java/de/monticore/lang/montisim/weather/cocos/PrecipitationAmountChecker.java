/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for SimulationLoopFrequency
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.weather._ast.ASTPrecipitationAmount;
import de.monticore.lang.montisim.weather._cocos.WeatherASTPrecipitationAmountCoCo;
import de.monticore.lang.montisim.weather.symboltable.PrecipitationAmountSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class PrecipitationAmountChecker implements WeatherASTPrecipitationAmountCoCo {
  
  @Override
  public void check(ASTPrecipitationAmount node) {
    String[] allowedUnits = {"l/m^2", "mm"};
    PrecipitationAmountSymbol sym = (PrecipitationAmountSymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getPrecipitationAmount()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.warn("Unit Error: precipitation_amount invalid or missing unit.");
      }
      if (!checker.inMinRange(0)) {
        Log.warn("Range Error: precipitation_amount must be at least 0.");
      }
    }
  }
  
}
