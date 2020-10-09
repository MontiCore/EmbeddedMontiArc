/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.monticar.common2._ast.ASTParameter;
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
    implements EmbeddedMontiArcASTComponentCoCo {


  @Override
  public void check(ASTComponent node ) {
    List<ASTParameter> params = node.getParameterList();
    boolean foundDefaultParameter = false;
    for (ASTParameter param : params) {

      if (!foundDefaultParameter) {
        foundDefaultParameter = param.getDefaultValueOpt().isPresent();
      }
      else {
        if (foundDefaultParameter && !param.getDefaultValueOpt().isPresent()) {
          Log.error("0xAC005 There are non default parameters after a default parameter",
              node.get_SourcePositionStart());
        }
      }
    }

  }

}
