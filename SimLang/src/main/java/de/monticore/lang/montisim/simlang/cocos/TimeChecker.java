/*
 * Custom CoCos for Time
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTSingleTime;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTSingleTimeCoCo;
import de.monticore.lang.montisim.weather.cocos.NumberUnit;
import de.monticore.lang.montisim.weather.cocos.UnitNumberChecker;
import de.monticore.lang.numberunit._ast.ASTUnitNumber;
import de.se_rwth.commons.logging.Log;
import java.util.Optional;

public class TimeChecker implements SimLangASTSingleTimeCoCo {
  
  @Override
  public void check(ASTSingleTime timeObj) {
    String[] allowedUnits = {""};
    
    NumberUnit timeHours = new NumberUnit(timeObj.getHours());
    UnitNumberChecker checkerHours = new UnitNumberChecker(timeHours,allowedUnits);
    NumberUnit timeMinutes = new NumberUnit(timeObj.getMinutes());
    UnitNumberChecker checkerMinutes = new UnitNumberChecker(timeMinutes,allowedUnits);
    
    if(!checkerHours.legitUnit() || !checkerMinutes.legitUnit()) {
      Log.error("Unit Error: Time does not take units.");
      return;
    }
    if(!checkerHours.noDecimals() || !checkerMinutes.noDecimals()) {
      Log.error("Type Error: Time does not take decimals.");
      return;
    }
    int hours = (int)timeHours.getNumber();
    int minutes = (int)timeMinutes.getNumber();
    
    Optional<ASTUnitNumber> timeSeconds = timeObj.getSeconds();
    Optional<ASTUnitNumber> timeMilliseconds = timeObj.getMilliseconds();
    
    if(hours < 0 || 23 < hours) {
      Log.error("Range Error: Time:hours must be within [0,23].");
    }
    if(minutes < 0 || 59 < minutes) {
      Log.error("Range Error: Time:minutes must be within [0,59].");
    }
    if(timeSeconds.isPresent()) {
      UnitNumberChecker checkerSeconds = new UnitNumberChecker(new NumberUnit(timeSeconds.get()) ,allowedUnits);
      if(!checkerSeconds.legitUnit()) {
      Log.error("Unit Error: Time does not take units.");
      return;
      }
      if(!checkerSeconds.noDecimals()) {
        Log.error("Type Error: Time does not take decimals.");
      }
      int seconds = timeSeconds.get().getNumber().get().intValue();
      if(seconds < 0 || 59 < seconds) {
        Log.error("Range Error: Time:seconds must be within [0,59].");
      }
    }
    if(timeMilliseconds.isPresent()) {
      UnitNumberChecker checkerMilliseconds = new UnitNumberChecker(new NumberUnit(timeMilliseconds.get()),allowedUnits);
      if(!checkerMilliseconds.legitUnit()) {
      Log.error("Unit Error: Time does not take units.");
      return;
      }
      if(!checkerMilliseconds.noDecimals()) {
        Log.error("Type Error: Time does not take decimals.");
      }
      int milliseconds = timeMilliseconds.get().getNumber().get().intValue();
      if(milliseconds < 0 || 999 < milliseconds) {
        Log.error("Range Error: Time:milliseconds must be within [0,999].");
      }
    }
  }
  
}
