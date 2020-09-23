/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTComponent;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentCoCo;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewComponentSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

/**
 */
public class ComponentWithTypeParametersHasInstance
    implements EmbeddedMontiViewASTComponentCoCo {

  /**
   * @see EmbeddedMontiViewASTComponentCoCo#check(ASTComponent)
   */
  @Override
  public void check(ASTComponent node) {
    ViewComponentSymbol viewComponentSymbol = (ViewComponentSymbol) node.getSymbol().get();

    Collection<ViewComponentInstanceSymbol> subComponents = viewComponentSymbol.getSubComponents();

    Set<ViewComponentSymbol> instantiatedInnerComponents = subComponents.stream().map(instanceSymbol -> instanceSymbol.getComponentType().getReferencedSymbol()).filter(symbol -> symbol.hasFormalTypeParameters()).collect(Collectors.toSet());

    List<ViewComponentSymbol> notInstantiatedInnerComponents = viewComponentSymbol.getInnerComponents().stream().filter(symbol -> symbol.hasFormalTypeParameters()).filter(innerComponent -> !instantiatedInnerComponents.contains(innerComponent)).collect(Collectors.toList());

    for (ViewComponentSymbol notInstantiatedInnerComponent : notInstantiatedInnerComponents) {
      Log.error(String.format("0x79C00 Inner component \"%s\" must have an instance defining its formal type parameters.", notInstantiatedInnerComponent.getName()), notInstantiatedInnerComponent.getSourcePosition());
    }
  }
}
