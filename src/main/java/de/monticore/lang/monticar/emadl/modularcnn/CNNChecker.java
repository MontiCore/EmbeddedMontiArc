package de.monticore.lang.monticar.emadl.modularcnn;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.se_rwth.commons.logging.Log;

public class CNNChecker {
    
    public CNNChecker() {
        
    }
    
    public void check(ASTComponent component){
        ComponentInformation componentInformation = new ComponentInformation(component);
        Log.info(component.getName() + " is composed CNN: " + componentInformation.isComposedCNN(),"COMPOSED_CNN");


        
    }
    
    
    
}
