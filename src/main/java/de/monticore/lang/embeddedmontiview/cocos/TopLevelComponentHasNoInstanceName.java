/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTComponent;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentCoCo;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewComponentSymbol;
import de.se_rwth.commons.logging.Log;

/**
 */
public class TopLevelComponentHasNoInstanceName
    implements EmbeddedMontiViewASTComponentCoCo {

  /**
   * @see EmbeddedMontiViewASTComponentCoCo#check(ASTComponent)
   */
  @Override
  public void check(ASTComponent node) {
    if (!node.symbolIsPresent()) {
      Log.error(String.format("0xE51E8 Symbol of component \"%s\" is missing. " + "The context condition \"%s\" can't be checked that way.", node.getName(), TopLevelComponentHasNoInstanceName.class.getName()));
    }

    ViewComponentSymbol symbol = (ViewComponentSymbol) node.getSymbol().get();
    if (!symbol.isInnerComponent() && node.instanceNameIsPresent()) {
      Log.error(String.format("0x3F207 Top level component \"%s\" has an instance name", node.getName()), node.get_SourcePositionStart());
    }
  }

}
