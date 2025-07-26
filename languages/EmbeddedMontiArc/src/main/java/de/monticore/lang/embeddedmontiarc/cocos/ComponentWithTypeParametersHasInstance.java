/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

/**
 */
public class ComponentWithTypeParametersHasInstance
    implements EmbeddedMontiArcASTComponentCoCo {

  /**
   * @see EmbeddedMontiArcASTComponentCoCo#check(ASTComponent)
   */
  @Override
  public void check(ASTComponent node) {
    EMAComponentSymbol emaComponentSymbol = (EMAComponentSymbol) node.getSymbolOpt().get();

    Collection<EMAComponentInstantiationSymbol> subComponents = emaComponentSymbol.getSubComponents();

    Set<EMAComponentSymbol> instantiatedInnerComponents = subComponents
        .stream()
        .map(instanceSymbol -> instanceSymbol.getComponentType().getReferencedSymbol())
        .filter(symbol -> symbol.hasFormalTypeParameters())
        .collect(Collectors.toSet());

    List<EMAComponentSymbol> notInstantiatedInnerComponents = emaComponentSymbol
        .getInnerComponents()
        .stream()
        .filter(symbol -> symbol.hasFormalTypeParameters())
        .filter(innerComponent -> !instantiatedInnerComponents.contains(innerComponent))
        .collect(Collectors.toList());

    for (EMAComponentSymbol notInstantiatedInnerComponent : notInstantiatedInnerComponents) {
      Log.error(
          String.format(
              "0x79C00 Inner component \"%s\" must have an instance defining its formal type parameters.",
              notInstantiatedInnerComponent.getName()),
          notInstantiatedInnerComponent.getSourcePosition());
    }
  }
}
