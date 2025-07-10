/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTComponentHead;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentHeadCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 * Ensures that parameters in the component's head are defined in the right order.
 * It is not allowed to define a normal parameter after a declaration of a default parameter.
 * E.g.: Wrong: A[int x = 5, int y]
 * Right: B[int x, int y = 5]
 *
 */
public class DefaultParametersHaveCorrectOrder
    implements EmbeddedMontiViewASTComponentHeadCoCo {

  /**
   * @see EmbeddedMontiViewASTComponentHeadCoCo#check(ASTComponentHead)
   */
  @Override
  public void check(ASTComponentHead node) {
    List<ASTParameter> params = node.getParameters();
    boolean foundDefaultParameter = false;
    for (ASTParameter param : params) {

      if (!foundDefaultParameter) {
        foundDefaultParameter = param.getDefaultValue().isPresent();
      }
      else {
        if (foundDefaultParameter && !param.getDefaultValue().isPresent()) {
          Log.error("0xAC005 There are non default parameters after a default parameter", node.get_SourcePositionStart());
        }
      }
    }

  }

}
