/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTComponentHead;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentHeadCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class ParameterNamesUnique
    implements EmbeddedMontiViewASTComponentHeadCoCo {

  /**
   * @see EmbeddedMontiViewASTComponentHeadCoCo#check(ASTComponentHead)
   */
  @Override
  public void check(ASTComponentHead node) {
    List<ASTParameter> parameters = node.getParameters();

    List<String> parameterNames = new ArrayList<>();
    for (ASTParameter parameter : parameters) {

      if (parameterNames.contains(parameter.getName())) {
        Log.error(String.format("0xC4A61 Parameter name \"%s\" not unique", parameter.getName()), parameter.get_SourcePositionStart());
      }

      else {
        parameterNames.add(parameter.getName());
      }
    }
  }

}
