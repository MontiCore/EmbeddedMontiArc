/*
 * Custom CoCos for Time
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package simlang.cocos;

import simlang._ast.ASTTime;
import si._ast.ASTUnitNumber;
import simlang._cocos.SimLangASTTimeCoCo;
import de.se_rwth.commons.logging.Log;
import simlang.cocos.UnitNumberChecker;
import java.util.Optional;

public class TimeChecker implements SimLangASTTimeCoCo {
  
  @Override
  public void check(ASTTime timeObj) {
    System.out.println("[CoCo] TimeChecker...");
    
    String[] allowedUnits = {""};
    
    String timeHours = timeObj.getTimeHours();
    UnitNumberChecker checkerHours = new UnitNumberChecker(timeHours,allowedUnits);
    String timeMinutes = timeObj.getTimeMinutes();
    UnitNumberChecker checkerMinutes = new UnitNumberChecker(timeMinutes,allowedUnits);
    
    if(!checkerHours.legitUnit() || !checkerMinutes.legitUnit()) {
      Log.error("Unit Error: Time does not take units.");
    }
    if(!checkerHours.noDecimals() || !checkerMinutes.noDecimals()) {
      Log.error("Type Error: Time does not take decimals.");
    }
    int hours = (int)Float.parseFloat(timeHours);
    int minutes = (int)Float.parseFloat(timeMinutes);
    
    Optional<String> timeSeconds = timeObj.getTimeSeconds();
    Optional<String> timeMilliseconds = timeObj.getTimeMilliseconds();
    
    if(hours < 0 || 23 < hours) {
      Log.error("Range Error: Time:hours must be within [0,23].");
    }
    if(minutes < 0 || 59 < minutes) {
      Log.error("Range Error: Time:minutes must be within [0,59].");
    }
    if(timeSeconds.isPresent()) {
      UnitNumberChecker checkerSeconds = new UnitNumberChecker(timeSeconds.get(),allowedUnits);
      if(!checkerSeconds.legitUnit()) {
      Log.error("Unit Error: Time does not take units.");
      }
      if(!checkerSeconds.noDecimals()) {
        Log.error("Type Error: Time does not take decimals.");
      }
      int seconds = (int)Float.parseFloat(timeSeconds.get());
      if(seconds < 0 || 59 < seconds) {
        Log.error("Range Error: Time:seconds must be within [0,59].");
      }
    }
    if(timeMilliseconds.isPresent()) {
      UnitNumberChecker checkerMilliseconds = new UnitNumberChecker(timeMilliseconds.get(),allowedUnits);
      if(!checkerMilliseconds.legitUnit()) {
      Log.error("Unit Error: Time does not take units.");
      }
      if(!checkerMilliseconds.noDecimals()) {
        Log.error("Type Error: Time does not take decimals.");
      }
      int milliseconds = (int)Float.parseFloat(timeMilliseconds.get());
      if(milliseconds < 0 || 999 < milliseconds) {
        Log.error("Range Error: Time:milliseconds must be within [0,999].");
      }
    }
    
    System.out.println("[Done] TimeChecker");
  }
  
}
