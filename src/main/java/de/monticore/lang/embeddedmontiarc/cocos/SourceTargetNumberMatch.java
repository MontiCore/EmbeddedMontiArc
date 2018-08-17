/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
