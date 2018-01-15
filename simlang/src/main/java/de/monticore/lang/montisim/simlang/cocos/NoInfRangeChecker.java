/*
 * Custom CoCos for Gravity
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

//import ranges._ast.ASTRange;
//import ranges._cocos.RangesASTRangeCoCo;
import de.se_rwth.commons.logging.Log;
import java.util.Optional;

public class NoInfRangeChecker implements RangesASTRangeCoCo {
  
  @Override
  public void check(ASTRange obj) {
    System.out.println("[CoCo] NoInfRangeChecker...");
        
    if(obj.getStartInf().isPresent() || obj.getEndInf().isPresent()) {
      Log.error("Range Error: Infinite start/end are not allowed.");
    }
    
    System.out.println("[Done] NoInfRangeChecker");
  }
  
}
