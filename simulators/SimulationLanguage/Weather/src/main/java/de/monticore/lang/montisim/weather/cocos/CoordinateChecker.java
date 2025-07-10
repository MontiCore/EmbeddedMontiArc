/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for Coordinate
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.weather._ast.ASTCoordinate;
import de.monticore.lang.montisim.weather._cocos.WeatherASTCoordinateCoCo;
import de.se_rwth.commons.logging.Log;
import de.monticore.lang.montisim.util.types.NumberUnit;

public class CoordinateChecker implements WeatherASTCoordinateCoCo {
  
  @Override
  public void check(ASTCoordinate node) {
    String[] allowedUnits = {""};

    NumberUnit inputX = new NumberUnit(node.getPosX());
    NumberUnit inputY = new NumberUnit(node.getPosY());
    
    NumberUnitChecker checkerX = new NumberUnitChecker(inputX, allowedUnits);
    NumberUnitChecker checkerY = new NumberUnitChecker(inputY, allowedUnits);
    
    if(!checkerX.legitUnit() || !checkerY.legitUnit()) {
      Log.warn("Unit Error: Coordinate invalid unit.");
    }
  }
  
}
