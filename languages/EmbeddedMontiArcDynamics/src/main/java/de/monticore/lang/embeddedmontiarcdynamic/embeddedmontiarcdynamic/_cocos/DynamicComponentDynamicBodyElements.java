/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTInterface;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTEventHandler;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTPort;
import de.se_rwth.commons.logging.Log;

public class DynamicComponentDynamicBodyElements implements EmbeddedMontiArcDynamicASTComponentCoCo {
    @Override
    public void check(ASTComponent node) {

        boolean isDynamic = node.isDynamic();

        for (ASTElement ele : node.getBody().getElementList()){
            //Check Interface
            if(ele instanceof ASTInterface){
                ASTInterface ports = (ASTInterface)ele;
                for(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort port : ports.getPortsList()){
                    if(port instanceof ASTPort){
                        ASTPort p = (ASTPort)port;
                        if(!isDynamic && p.isDynamic()){
                            Log.error(String.format("0xAD002 Port '%s' is dynamic, but the component '%s' is not dynamic!",
                                    port.getName(), node.getName()), port.get_SourcePositionStart());

                        }
                    }
                }
            }

            if(ele instanceof ASTEventHandler){
                //more checks

            }
        }
    }
}
