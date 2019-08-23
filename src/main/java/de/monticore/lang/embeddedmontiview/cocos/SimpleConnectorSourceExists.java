/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTComponent;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentCoCo;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.*;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

/**
 * Implementation of R7
 *
 * @author Crispin Kirchner
 */
public class SimpleConnectorSourceExists
    implements EmbeddedMontiViewASTComponentCoCo {

  /**
   * TODO: either check why ViewConnectorSymbol has no proper value for sourcePosition, or reimplement
   * using
   *
   * @see EmbeddedMontiViewASTComponentCoCo#check(ASTComponent)
   */
  @Override
  public void check(ASTComponent node) {
    ViewComponentSymbol symbol = (ViewComponentSymbol) node.getSymbol().orElse(null);

    if (null == symbol) {
      Log.error(String.format("0x9AF6C ASTComponent node \"%s\" has no symbol. Did you forget to " + "run the SymbolTableCreator before checking cocos?", node.getName()));
      return;
    }

    for (ViewComponentInstanceSymbol instanceSymbol : symbol.getSubComponents()) {
      for (ViewConnectorSymbol viewConnectorSymbol : instanceSymbol.getSimpleConnectors()) {

        ViewComponentSymbolReference typeReference = instanceSymbol.getComponentType();

        if (!typeReference.existsReferencedSymbol()) {
          Log.error(String.format("0xBEA8B The component type \"%s\" can't be resolved.", typeReference.getFullName()));
          return;
        }

        ViewComponentSymbol sourceComponent = typeReference.getReferencedSymbol();
        String sourcePort = viewConnectorSymbol.getSource();

        Optional<ViewPortSymbol> outgoingPort = sourceComponent.getOutgoingPort(sourcePort);

        if (!outgoingPort.isPresent()) {
          Log.error(String.format("0xF4D71 Out port \"%s\" is not present in component \"%s\".", sourcePort, sourceComponent.getName()), viewConnectorSymbol.getSourcePosition());
        }
      }
    }
  }

}
