/*
 * Custom CoCos for Area
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTArea;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTAreaCoCo;

import de.se_rwth.commons.logging.Log;

import java.util.Optional;

public class AreaChecker implements SimLangASTAreaCoCo {
  
  @Override
  public void check(ASTArea obj) {
    System.out.println("[CoCo] AreaChecker...");
    
    if(obj.isGlobal() || !obj.getRadius().isPresent()) {
      System.out.println("[Done] AreaChecker");
      return;
    }
    
    String[] allowedUnits = {""};
    
    String input = obj.getRadius().get();
    
    UnitNumberChecker checker = new UnitNumberChecker(input, allowedUnits);
    
    if(!checker.inMinRange(0)) {
      Log.error("Range Error: Area must be at least 0.");
    }
    if(!checker.legitUnit()) {
      Log.error("Unit Error: Area missing or invalid unit.");
    }
    
    //Coordinates are handled by the Coordinate CoCo
    
    System.out.println("[Done] AreaChecker");
  }
  
}
