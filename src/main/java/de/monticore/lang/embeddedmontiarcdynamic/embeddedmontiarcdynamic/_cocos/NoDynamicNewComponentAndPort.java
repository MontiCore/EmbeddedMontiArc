/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTQualifiedNameWithArrayAndStar;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTConnectorCoCo;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTEventHandler;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicPortHelper;
import de.se_rwth.commons.logging.Log;

import java.util.List;


public class NoDynamicNewComponentAndPort implements EmbeddedMontiArcASTConnectorCoCo{

//    @Override
//    public void check(ASTConnector astConnector) {
//        if(astConnector.isPresentSource()){
//            if(astConnector.getSource().getQualifiedNameWithArray() instanceof ASTQualifiedNameWithArray){
//                ASTQualifiedNameWithArray a = (ASTQualifiedNameWithArray)astConnector.getSource().getQualifiedNameWithArray();
//                if(a.isPresentDynamicNewPort() && a.isPresentDynamicNewComponent()) {
//                    Log.error("0xAD004 New component AND port are not allowed (source of connect)!!", a.get_SourcePositionStart());
//                }
//            }
//        }
//
//        astConnector.getTargets().streamQualifiedNameWithArrayAndStars().forEach(t -> {
//            if(t.getQualifiedNameWithArray() instanceof ASTQualifiedNameWithArray){
//                ASTQualifiedNameWithArray b = (ASTQualifiedNameWithArray)t.getQualifiedNameWithArray();
//                if(b.isPresentDynamicNewPort() && b.isPresentDynamicNewComponent()) {
//                    Log.error("0xAD005 New component AND port are not allowed (target of connect)!!", b.get_SourcePositionStart());
//                }
//            }
//        });


    @Override
    public void check(ASTConnector connector) {
        if (connector.getSourceOpt().isPresent()) {
            if(connector.getSource().isPresentDotStar()) {
                Log.error("TODO: star connector setup");
//                        EMADynamicPortHelper.starConnectorSetup(node, this);
            } else {
                for(String s : EMADynamicPortHelper.getINSTANCE().getPortNameWithDynamic(connector.getSource(), null)){
                    if(s.chars().filter(ch -> ch =='?').count() > 1){
                        Log.error("0xAD004 New component AND port are not allowed (source of connect)!!", connector.get_SourcePositionStart());
                    }
                }
            }
        }
        for (ASTQualifiedNameWithArrayAndStar target : connector.getTargets().getQualifiedNameWithArrayAndStarList()) {
            for(String t : EMADynamicPortHelper.getINSTANCE().getPortNameWithDynamic(target, null)){
                if(t.chars().filter(ch -> ch =='?').count() > 1){
                    Log.error("0xAD005 New component AND port are not allowed (target of connect)!!", connector.get_SourcePositionStart());
                }
            }

        }
    }
}
