/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTConnector;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTConnectorCoCo;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.se_rwth.commons.logging.Log;

import java.util.function.Predicate;

/**
 *         Implementation of CO1 and CO2
 */
public class ConnectorEndPointCorrectlyQualified
    implements EmbeddedMontiViewASTConnectorCoCo {

  private void checkEndpointCorrectlyQualified(ASTQualifiedNameWithArray name, Predicate<Integer> predicate, String errorMessage) {
    int i = 0;
    if (name.getCompName() != null)
      ++i;
    if (name.getPortName() != null)
      ++i;
    //if (!predicate.test(name.getParts().size())) {
    if (!predicate.test(i)) {
      Log.error(String.format(errorMessage, name.toString()), name.get_SourcePositionStart());
    }
  }

  /**
   * Ensure that the connector endpoint is of the form `rootComponentPort' or `subComponent.port'
   */
  private void checkEndPointMaximallyTwiceQualified(ASTQualifiedNameWithArray name) {
    checkEndpointCorrectlyQualified(name, i -> i <= 2 && i > 0, "0xDB61C Connector endVal point \"%s\" must only consist of an optional component name and a port name");
  }

  /**
   * @see EmbeddedMontiViewASTConnectorCoCo#check(ASTConnector)
   */
  @Override
  public void check(ASTConnector node) {
    //    if(node.getSource().isPresent()) {
    checkEndPointMaximallyTwiceQualified(node.getSource());//.get());
    //    }else{
    //      Log.error("Error Connector has no valid source or constant");
    //    }

    for (ASTQualifiedNameWithArray name : node.getTargets()) {
      checkEndPointMaximallyTwiceQualified(name);
    }
  }

}
