/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTQualifiedNameWithArrayAndStar;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTConnectorCoCo;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicPortHelper;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.se_rwth.commons.logging.Log;

import java.util.List;

public class NoDynamicNewConnectsOutsideEventHandler implements EmbeddedMontiArcDynamicASTComponentCoCo {
//EmbeddedMontiArcDynamicASTComponentCoCo
//    @Override
//    public void check(ASTComponent node) {
//        for (ASTElement ele : node.getBody().getElementList()) {
//
//            if (ele instanceof ASTConnector) {
//                ASTConnector connector = (ASTConnector)ele;
//                if(connector.isPresentSource()){
//                    if(connector.getSource().getQualifiedNameWithArray() instanceof ASTQualifiedNameWithArray){
//                        ASTQualifiedNameWithArray a = (ASTQualifiedNameWithArray)connector.getSource().getQualifiedNameWithArray();
//                        if(a.isPresentDynamicNewPort() || a.isPresentDynamicNewComponent()) {
//                            Log.error("0xAD003 Dynamic connect outside of event handler (source of connect)!!", a.get_SourcePositionStart());
//                        }
//                    }
//                }
//
//                connector.getTargets().forEachQualifiedNameWithArrayAndStars(obj -> {
//                    if (obj.getQualifiedNameWithArray() instanceof ASTQualifiedNameWithArray) {
//                        ASTQualifiedNameWithArray t = (ASTQualifiedNameWithArray) obj.getQualifiedNameWithArray();
//                        if (t.isPresentDynamicNewPort() || t.isPresentDynamicNewComponent()) {
//                            Log.error(String.format("0xAD003 Dynamic connect outside of event handler (target of connect)!!",
//                                    t.getPortName(), node.getName()), t.get_SourcePositionStart());
//                        }
//                    }
//                });
//
//            }
//        }
//    }

    @Override
    public void check(ASTComponent node) {
        for (ASTElement ele : node.getBody().getElementList()) {
            if (ele instanceof ASTConnector) {
                ASTConnector connector = (ASTConnector)ele;

                if (connector.getSourceOpt().isPresent()) {
                    if(connector.getSource().isPresentDotStar()) {
                        Log.error("TODO: star connector setup");
//                        EMADynamicPortHelper.starConnectorSetup(node, this);
                    } else {
                        for(String s : EMADynamicPortHelper.getINSTANCE().getPortNameWithDynamic(connector.getSource(), null)){
                            if(s.contains("?")){
                                Log.error("0xAD003 Dynamic connect outside of event handler (source of connect)!!", connector.get_SourcePositionStart());
                            }
                        }
                    }
                }
                for (ASTQualifiedNameWithArrayAndStar target : connector.getTargets().getQualifiedNameWithArrayAndStarList()) {
                    for(String t : EMADynamicPortHelper.getINSTANCE().getPortNameWithDynamic(target, null)){
                        if(t.contains("?")) {
                            Log.error(String.format("0xAD003 Dynamic connect (%s), with dynamic port %s, outside of event handler (target of connect)!!",
                                    node.getName(), t), connector.get_SourcePositionStart());
                        }
                    }

                }


            }
        }
    }
}

