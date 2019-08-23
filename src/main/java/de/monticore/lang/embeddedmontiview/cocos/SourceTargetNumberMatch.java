/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTConnector;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTConnectorCoCo;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.se_rwth.commons.logging.Log;

/**
 * Created by Sining on 2017/3/5.
 */
public class SourceTargetNumberMatch
    implements EmbeddedMontiViewASTConnectorCoCo {
  @Override
  public void check(ASTConnector node) {
    int sourceNum = 0, targetNum = 0;

    sourceNum = getSourceNum(node);

    for (ASTQualifiedNameWithArray target : node.getTargets()) {

      targetNum = getTargetNum(target);

      if (sourceNum != targetNum) {
        Log.error("0xJK901 source port number " + sourceNum + " and target port number " + targetNum + " don't match");
      }
    }
  }

  private int getSourceNum(ASTConnector node) {
    int sourceNum = 0, sourceComp = 0, sourcePort = 0;
    //        if (node.getSource().isPresent()) {
    if (node.getSource().getCompArray().isPresent()) {//.get().getCompArray().isPresent()){
      if (node.getSource().getCompArray().get().getLowerbound().isPresent())//.get().getCompArray().get().getLowerbound().isPresent())
        sourceComp = node.getSource().getCompArray().get().getUpperbound().get().getNumber().get().intValue() - node.getSource().getCompArray().get().getLowerbound().get().getNumber().get().intValue() + 1;
    }
    else
      sourceComp = 1;
    if (node.getSource().getPortArray().isPresent()) {//.get().getPortArray().isPresent()){
      if (node.getSource().getPortArray().get().getLowerbound().isPresent())//.get().getPortArray().get().getLowerbound().isPresent())
        sourcePort = node.getSource().getPortArray().get().getUpperbound().get().getNumber().get().intValue() - node.getSource().getPortArray().get().getLowerbound().get().getNumber().get().intValue() + 1;
    }
    else
      sourcePort = 1;
    sourceNum = sourceComp * sourcePort;
    //        }
    return sourceNum;
  }

  private int getTargetNum(ASTQualifiedNameWithArray target) {
    int targetNum = 0, targetComp = 0, targetPort = 0;
    if (target.getCompArray().isPresent()) {
      if (target.getCompArray().get().getLowerbound().isPresent())
        targetComp = target.getCompArray().get().getUpperbound().get().getNumber().get().intValue() - target.getCompArray().get().getLowerbound().get().getNumber().get().intValue() + 1;
    }
    else
      targetComp = 1;
    if (target.getPortArray().isPresent()) {
      if (target.getPortArray().get().getLowerbound().isPresent())
        targetPort = target.getPortArray().get().getUpperbound().get().getNumber().get().intValue() - target.getPortArray().get().getLowerbound().get().getNumber().get().intValue() + 1;
    }
    else
      targetPort = 1;
    targetNum = targetComp * targetPort;
    return targetNum;
  }
}
