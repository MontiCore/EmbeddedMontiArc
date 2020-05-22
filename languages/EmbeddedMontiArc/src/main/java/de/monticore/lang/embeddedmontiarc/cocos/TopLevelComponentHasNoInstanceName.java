/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.se_rwth.commons.logging.Log;

/**
 */
public class TopLevelComponentHasNoInstanceName
    implements EmbeddedMontiArcASTComponentCoCo {
  
  /**
   * @see EmbeddedMontiArcASTComponentCoCo#check(ASTComponent)
   */
  @Override
  public void check(ASTComponent node) {
    if (!node.getSymbolOpt().isPresent()) {
      Log.error(String.format(
          "0xE51E8 Symbol of component \"%s\" is missing. " +
              "The context condition \"%s\" can't be checked that way.",
          node.getName(), TopLevelComponentHasNoInstanceName.class.getName()));
    }
    
  }
}
