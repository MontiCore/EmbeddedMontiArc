/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for Timeout
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTTransferRate;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTTransferRateCoCo;
import de.monticore.lang.montisim.simlang._symboltable.TransferRateSymbol;
import de.monticore.lang.montisim.util.cocos.NumberUnitChecker;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.monticore.lang.montisim.util.cocos.InputHelper;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class TransferrateChecker implements SimLangASTTransferRateCoCo {
  
  @Override
  public void check(ASTTransferRate node) {
    String[] allowedUnits = {"bit/s","Kbit/s","Mbit/s","Gbit/s"};
    TransferRateSymbol sym = (TransferRateSymbol)node.getSymbol().get();
    ArrayList<NumberUnit> input = new InputHelper(sym.getTransferRate()).getExtractedValues();

    for(NumberUnit nu : input) {
      NumberUnitChecker checker = new NumberUnitChecker(nu, allowedUnits);

      if (!checker.inPositiveRange()) {
        Log.warn("Range Error: transfer_rate must be greater 0.");
      }
      if (!checker.legitUnit()) {
        Log.warn("Unit Error: transfer_rate missing or invalid unit.");
      }
    }
  }
  
}
