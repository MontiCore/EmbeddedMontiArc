/*
 * Custom CoCos for Timeout
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTTransferRate;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTTransferRateCoCo;
import de.monticore.lang.montisim.weather.cocos.InputHelper;
import de.monticore.lang.montisim.weather.cocos.UnitNumberChecker;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Optional;

public class TransferrateChecker implements SimLangASTTransferRateCoCo {
  
  @Override
  public void check(ASTTransferRate obj) {
    System.out.println("[CoCo] TransferRateChecker...");
    
    String[] allowedUnits = {"bit/s","Kbit/s","Mbit/s","Gbit/s"};
    ArrayList<String> input = new InputHelper(obj.getAlternativeInput()).getExtractedValues();

    for(String nu : input) {
      UnitNumberChecker checker = new UnitNumberChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.error("Range Error: transfer_rate must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.error("Unit Error: transfer_rate missing or invalid unit.");
      }
    }
    System.out.println("[Done] TransferRateChecker");
  }
  
}
