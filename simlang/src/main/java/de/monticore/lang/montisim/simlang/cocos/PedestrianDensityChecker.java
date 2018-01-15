/*
 * Custom CoCos for PedestrianDensity
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package de.monticore.lang.montisim.simlang.cocos;

import de.monticore.lang.montisim.simlang._ast.ASTPedestrianDensity;
import de.monticore.lang.montisim.simlang._cocos.SimLangASTPedestrianDensityCoCo;
import de.se_rwth.commons.logging.Log;
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
