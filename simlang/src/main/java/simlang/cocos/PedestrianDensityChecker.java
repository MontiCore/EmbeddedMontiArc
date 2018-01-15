/*
 * Custom CoCos for PedestrianDensity
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package simlang.cocos;

import simlang._ast.ASTPedestrianDensity;
import si._ast.ASTUnitNumber;
import simlang._cocos.SimLangASTPedestrianDensityCoCo;
import de.se_rwth.commons.logging.Log;
import simlang.cocos.UnitNumberChecker;
import java.util.Optional;

public class PedestrianDensityChecker implements SimLangASTPedestrianDensityCoCo {
  
  @Override
  public void check(ASTPedestrianDensity obj) {
    System.out.println("[CoCo] PedestrianDensityChecker...");
    
    float input = Float.parseFloat(obj.getPedestrianDensity());
    
    if(input < 0) {
      Log.error("Range Error: Pedestrian Density must be at least 0.");
    }
    
    System.out.println("[Done] PedestrianDensityChecker");
  }
  
}
