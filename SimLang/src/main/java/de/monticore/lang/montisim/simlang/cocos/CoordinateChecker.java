/*
 * Custom CoCos for Coordinate
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTCoordinate;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTCoordinateCoCo;

import de.se_rwth.commons.logging.Log;

public class CoordinateChecker implements SimLangASTCoordinateCoCo {
  
  @Override
  public void check(ASTCoordinate obj) {
    System.out.println("[CoCo] CoordinateChecker...");
    
    String[] allowedUnits = {""};
    
    String inputX = obj.getPosX();
    String inputY = obj.getPosY();
    
    UnitNumberChecker checkerX = new UnitNumberChecker(inputX, allowedUnits);
    UnitNumberChecker checkerY = new UnitNumberChecker(inputY, allowedUnits);
    
    if(!checkerX.legitUnit() || !checkerY.legitUnit()) {
      Log.error("Unit Error: Coordinate invalid unit.");
    }
    
    //Coordinates are handled by the Coordinate CoCo
    
    System.out.println("[Done] CoordinateChecker");
  }
  
}
