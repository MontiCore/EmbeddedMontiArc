/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for SimulationLoopFrequency
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.weather._ast.ASTSight;
import de.monticore.lang.montisim.weather._cocos.WeatherASTSightCoCo;
import de.monticore.lang.montisim.weather.symboltable.SightSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class SightChecker implements WeatherASTSightCoCo {
  
  @Override
  public void check(ASTSight node) {
    String[] allowedUnits = {"mm","cm","dm","m","km"};
    SightSymbol sym = (SightSymbol)node.getSymbol().get();
    
    if(sym.getSight().isUnlimited()) {
      return;
    }
    ArrayList<NumberUnit> input = new InputHelper(sym.getSight().getSight().get()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.legitUnit()) {
        Log.warn("Unit Error: Sight invalid or missing unit.");
      }
      if (!checker.inMinRange(0)) {
        Log.warn("Range Error: Sight must be at least 0.");
      }
    }
  }
  
}
