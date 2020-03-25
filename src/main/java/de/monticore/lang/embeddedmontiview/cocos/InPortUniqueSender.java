/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.*;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentCoCo;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.monticore.types.types._ast.ASTQualifiedName;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * Implements R1 and R2
 *
 */
public class InPortUniqueSender implements EmbeddedMontiViewASTComponentCoCo {

  /**
   * @see EmbeddedMontiViewASTComponentCoCo#check(ASTComponent)
   */
  @Override
  public void check(ASTComponent node) {
    InPortUniqueSenderCheck check = new InPortUniqueSenderCheck(node);
    check.check();
  }

  private class InPortUniqueSenderCheck {
    private List<String> connectorTargets = new ArrayList<>();

    private ASTComponent node;

    public InPortUniqueSenderCheck(ASTComponent node) {
      this.node = node;
    }

    public void check() {
      checkConnectors();
      //checkSimpleConnectors();
    }

    private void checkTarget(ASTQualifiedNameWithArray target) {
      String targetString = target.toString();

      if (connectorTargets.contains(targetString)) {
        Log.error(String.format("0x2BD7E target port \"%s\" already in use.", target.toString()), target.get_SourcePositionStart());
      }
      else {
        connectorTargets.add(targetString);
      }
    }

    private void checkConnectors() {
      for (ASTConnector connector : node.getConnectors()) {
        for (ASTQualifiedNameWithArray target : connector.getTargets()) {
          checkTarget(target);
        }
      }
    }
    
    /*private void checkSimpleConnectors() {
      for (ASTSubComponent subComponent : node.getSubComponents()) {
        for (ASTSubComponentInstance instance : subComponent.getInstances()) {
          for (ASTSimpleConnector connector : instance.getConnectors()) {
            for (ASTQualifiedName target : connector.getTargets()) {
              checkTarget(target);
            }
          }
        }
      }
    }*/
  }

}
