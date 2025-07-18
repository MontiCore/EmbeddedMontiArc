/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for Area
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTArea;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTAreaCoCo;

import de.monticore.lang.montisim.simlang._symboltable.AreaSymbol;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.se_rwth.commons.logging.Log;

public class AreaChecker implements SimLangASTAreaCoCo {
  
  @Override
  public void check(ASTArea node) {
    AreaSymbol sym = (AreaSymbol)node.getSymbol().get();
    if(sym.getArea().isGlobal() || !sym.getArea().getRadius().isPresent()) {
      return;
    }
    
    String[] allowedUnits = {""};
    
    NumberUnitChecker checker = new NumberUnitChecker(sym.getArea().getRadius().get(), allowedUnits);
    
    if(!checker.inMinRange(0)) {
      Log.warn("Range Error: Area must be at least 0.");
    }
    if(!checker.legitUnit()) {
      Log.warn("Unit Error: Area missing or invalid unit.");
    }
    //Coordinates are handled by the Coordinate CoCo
  }
  
}
