/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTQualifiedNameWithArrayAndStar;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTConnectorCoCo;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.se_rwth.commons.logging.Log;

/**
 * Created by Sining on 2017/3/5.
 */
public class SourceTargetNumberMatch implements EmbeddedMontiArcASTConnectorCoCo {
    @Override
    public void check(ASTConnector node){
        int sourceNum = 0, targetNum = 0;

        sourceNum = getSourceNum(node);

        for (ASTQualifiedNameWithArrayAndStar target : node.getTargets().getQualifiedNameWithArrayAndStarList()) {

            targetNum = getTargetNum(target.getQualifiedNameWithArray());

            if (sourceNum != targetNum){
                Log.error("0xJK901 source port number "+ sourceNum +" and target port number "+ targetNum + " don't match");
            }
        }
    }

    private int getSourceNum(ASTConnector node){
        int sourceNum = 0, sourceComp = 0, sourcePort = 0;
        if (node.getSourceOpt().isPresent()) {
            ASTQualifiedNameWithArray source = node.getSource().getQualifiedNameWithArray();
            if (source.getCompArrayOpt().isPresent()){
                if (source.getCompArray().getLowerboundOpt().isPresent())
                    sourceComp = source.getCompArray().getUpperbound().getNumber().get().intValue() - source.getCompArray().getLowerbound().getNumber().get().intValue() + 1;
            }else sourceComp = 1;
            if (source.getPortArrayOpt().isPresent()){
                if (source.getPortArray().getLowerboundOpt().isPresent())
                    sourcePort = source.getPortArray().getUpperbound().getNumber().get().intValue() - source.getPortArray().getLowerbound().getNumber().get().intValue() + 1;
            }else sourcePort = 1;
            sourceNum = sourceComp * sourcePort;
        }
        return sourceNum;
    }

    private int getTargetNum(ASTQualifiedNameWithArray target){
        int targetNum = 0, targetComp = 0, targetPort = 0;
        if (target.getCompArrayOpt().isPresent()){
            if (target.getCompArray().getLowerboundOpt().isPresent())
                targetComp = target.getCompArray().getUpperbound().getNumber().get().intValue() - target.getCompArray().getLowerbound().getNumber().get().intValue() + 1;
        }else targetComp = 1;
        if (target.getPortArrayOpt().isPresent()){
            if (target.getPortArray().getLowerboundOpt().isPresent())
                targetPort = target.getPortArray().getUpperbound().getNumber().get().intValue() - target.getPortArray().getLowerbound().getNumber().get().intValue() + 1;
        }else targetPort = 1;
        targetNum = targetComp * targetPort;
        return targetNum;
    }
}
