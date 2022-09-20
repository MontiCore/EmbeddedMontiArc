package de.monticore.lang.monticar.emadl.modularcnn;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class CNNChecker {

    private ArrayList<ArchitectureNode> archNodes = null;
    public CNNChecker(ArrayList<ArchitectureNode> currentNodes) {
        this.archNodes = currentNodes;
    }
    
    public void check(ASTComponent component){
        ComponentInformation componentInformation = new ComponentInformation(component,archNodes);
        Log.info(component.getName() + " is composed CNN: " + componentInformation.isComposedCNN(),"COMPOSED_CNN");


        
    }
    
    
    
}
