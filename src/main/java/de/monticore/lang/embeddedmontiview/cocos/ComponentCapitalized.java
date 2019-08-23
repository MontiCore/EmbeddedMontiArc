/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTComponent;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentCoCo;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.EmbeddedMontiArcModelNameCalculator;
import de.se_rwth.commons.logging.Log;

/**
 * Ensures, that component names startVal in upper-case. This is required for inner components, see
 * {@link EmbeddedMontiArcModelNameCalculator}.
 *
 * @author Robert Heim
 */
public class ComponentCapitalized implements EmbeddedMontiViewASTComponentCoCo {

  /**
   * @see EmbeddedMontiViewASTComponentCoCo#check(ASTComponent)
   */
  @Override
  public void check(ASTComponent node) {
    if (!Character.isUpperCase(node.getName().charAt(0))) {
      Log.error("0xAC004 Component names must be startVal in upper-case", node.get_SourcePositionStart());
    }
  }
}
