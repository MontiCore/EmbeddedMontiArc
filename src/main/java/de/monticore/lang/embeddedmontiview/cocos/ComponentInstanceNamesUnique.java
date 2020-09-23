/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTComponent;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentCoCo;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewComponentSymbol;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * TODO: Write me!
 *
 */
public class ComponentInstanceNamesUnique
    implements EmbeddedMontiViewASTComponentCoCo {

  /**
   * @see EmbeddedMontiViewASTComponentCoCo#check(ASTComponent)
   */
  @Override
  public void check(ASTComponent node) {
    ViewComponentSymbol comp = (ViewComponentSymbol) node.getSymbol().get();
    List<String> names = new ArrayList<>();
    for (ViewComponentInstanceSymbol subComp : comp.getSubComponents()) {

      if (names.contains(subComp.getFullName())) {

        SourcePosition pos = subComp.getAstNode().isPresent() ? subComp.getAstNode().get().get_SourcePositionStart() : SourcePosition.getDefaultSourcePosition();

        Log.error(String.format("0xAC010 The subcomponent instance %s is not unique", subComp.getFullName()), pos);
      }
      else {
        names.add(subComp.getFullName());
      }
    }

  }
}
