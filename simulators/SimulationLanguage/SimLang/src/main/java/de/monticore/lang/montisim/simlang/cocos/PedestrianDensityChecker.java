/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for PedestrianDensity
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTPedestrianDensity;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTPedestrianDensityCoCo;
import de.monticore.lang.montisim.simlang._symboltable.PedestrianDensitySymbol;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class PedestrianDensityChecker implements SimLangASTPedestrianDensityCoCo {
  
  @Override
  public void check(ASTPedestrianDensity node) {
    String[] allowedUnits = {""};
    PedestrianDensitySymbol sym = (PedestrianDensitySymbol)node.getSymbol().get();

    ArrayList<NumberUnit> input = new InputHelper(sym.getPedestrianDensity()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.warn("Range Error: pedestrian_density must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.warn("Unit Error: pedestrian_density missing or invalid unit.");
      }
    }
  }
  
}
