/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class ParameterNamesUnique implements EmbeddedMontiArcASTComponentCoCo {


  public void check(ASTComponent node) {
    List<ASTParameter> parameters = node.getParameterList();

    List<String> parameterNames = new ArrayList<>();
    for (ASTParameter parameter : parameters) {

      if (parameterNames.contains(parameter.getNameWithArray().getName())) {
        Log.error(String.format("0xC4A61 Parameter name \"%s\" not unique", parameter.getNameWithArray().getName()),
            parameter.get_SourcePositionStart());
      }

      else {
        parameterNames.add(parameter.getNameWithArray().getName());
      }
    }
  }

}
