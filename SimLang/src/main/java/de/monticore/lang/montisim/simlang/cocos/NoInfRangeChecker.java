/* (c) https://github.com/MontiCore/monticore */
/*
 * Custom CoCos for Gravity
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.ranges._cocos.RangesASTRangeCoCo;
import de.se_rwth.commons.logging.Log;

public class NoInfRangeChecker implements RangesASTRangeCoCo {
  
  @Override
  public void check(ASTRange obj) {
    if(obj.hasNoLowerLimit() || obj.hasNoUpperLimit()) {
      Log.warn("Range Error: Infinite start/end are not allowed.");
    }
  }
  
}
