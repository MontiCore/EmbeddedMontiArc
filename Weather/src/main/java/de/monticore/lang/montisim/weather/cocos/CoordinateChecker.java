/*
 * Custom CoCos for Coordinate
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.weather._ast.ASTCoordinate;
import de.monticore.lang.montisim.weather._cocos.WeatherASTCoordinateCoCo;
import de.se_rwth.commons.logging.Log;

public class CoordinateChecker implements WeatherASTCoordinateCoCo {
  
  @Override
  public void check(ASTCoordinate obj) {
    String[] allowedUnits = {""};
    
    String inputX = obj.getPosX();
    String inputY = obj.getPosY();
    
    UnitNumberChecker checkerX = new UnitNumberChecker(inputX, allowedUnits);
    UnitNumberChecker checkerY = new UnitNumberChecker(inputY, allowedUnits);
    
    if(!checkerX.legitUnit() || !checkerY.legitUnit()) {
      Log.error("Unit Error: Coordinate invalid unit.");
    }
  }
  
}
