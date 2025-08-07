/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTComponentHead;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentHeadCoCo;
//import de.monticore.types.types._ast.ASTTypeParameters;
//import de.monticore.types.types._ast.ASTTypeVariableDeclaration;
import de.monticore.lang.monticar.types2._ast.ASTTypeParameters;
import de.monticore.lang.monticar.types2._ast.ASTTypeVariableDeclaration;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class TypeParameterNamesUnique
    implements EmbeddedMontiViewASTComponentHeadCoCo {

  /**
   * @see de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiViewASTComponentCoCo#check(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent)
   */
  @Override
  public void check(ASTComponentHead node) {
    ASTTypeParameters typeParameters = node.getGenericTypeParameters().orElse(null);
    if (typeParameters == null) {
      return;
    }

    List<String> typeParameterNames = new ArrayList<>();
    for (ASTTypeVariableDeclaration typeParameter : typeParameters.getTypeVariableDeclarations()) {

      if (typeParameterNames.contains(typeParameter.getNamingResolution().get().getName())) {
        Log.error(String.format("0x35F1A The formal type parameter name \"%s\" is not unique", typeParameter.getNamingResolution().get().getName()), typeParameter.get_SourcePositionStart());
      }

      else {
        typeParameterNames.add(typeParameter.getNamingResolution().get().getName());
      }
    }
  }

}
