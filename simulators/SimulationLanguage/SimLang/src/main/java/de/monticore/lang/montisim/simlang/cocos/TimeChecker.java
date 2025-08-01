/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for Time
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSingleTime;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSingleTimeCoCo;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.se_rwth.commons.logging.Log;
import java.util.Optional;

public class TimeChecker implements SimLangASTSingleTimeCoCo {
  
  @Override
  public void check(ASTSingleTime node) {
    String[] allowedUnits = {""};

    NumberUnit timeHours = new NumberUnit(node.getHours());
    NumberUnitChecker checkerHours = new NumberUnitChecker(timeHours,allowedUnits);
    NumberUnit timeMinutes = new NumberUnit(node.getMinutes());
    NumberUnitChecker checkerMinutes = new NumberUnitChecker(timeMinutes,allowedUnits);
    
    if(!checkerHours.legitUnit() || !checkerMinutes.legitUnit()) {
      Log.warn("Unit Error: Time does not take units.");
      return;
    }
    if(!checkerHours.noDecimals() || !checkerMinutes.noDecimals()) {
      Log.warn("Type Error: Time does not take decimals.");
      return;
    }
    int hours = (int)timeHours.getNumber();
    int minutes = (int)timeMinutes.getNumber();
    
    Optional<ASTNumberWithUnit> timeSeconds = node.getSecondsOpt();
    Optional<ASTNumberWithUnit> timeMilliseconds = node.getMillisecondsOpt();
    
    if(hours < 0 || 23 < hours) {
      Log.warn("Range Error: Time:hours must be within [0,23].");
    }
    if(minutes < 0 || 59 < minutes) {
      Log.warn("Range Error: Time:minutes must be within [0,59].");
    }
    if(timeSeconds.isPresent()) {
      NumberUnitChecker checkerSeconds = new NumberUnitChecker(new NumberUnit(timeSeconds.get()) ,allowedUnits);
      if(!checkerSeconds.legitUnit()) {
      Log.warn("Unit Error: Time does not take units.");
      return;
      }
      if(!checkerSeconds.noDecimals()) {
        Log.warn("Type Error: Time does not take decimals.");
      }
      int seconds = timeSeconds.get().getNumber().get().intValue();
      if(seconds < 0 || 59 < seconds) {
        Log.warn("Range Error: Time:seconds must be within [0,59].");
      }
    }
    if(timeMilliseconds.isPresent()) {
      NumberUnitChecker checkerMilliseconds = new NumberUnitChecker(new NumberUnit(timeMilliseconds.get()),allowedUnits);
      if(!checkerMilliseconds.legitUnit()) {
      Log.warn("Unit Error: Time does not take units.");
      return;
      }
      if(!checkerMilliseconds.noDecimals()) {
        Log.warn("Type Error: Time does not take decimals.");
      }
      int milliseconds = timeMilliseconds.get().getNumber().get().intValue();
      if(milliseconds < 0 || 999 < milliseconds) {
        Log.warn("Range Error: Time:milliseconds must be within [0,999].");
      }
    }
  }
  
}
