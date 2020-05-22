/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

/**
 * Implementation of R7
 *
 */
public class SimpleConnectorSourceExists implements EmbeddedMontiArcASTComponentCoCo {
  
  /**
   * TODO: either check why EMAConnectorSymbol has no proper value for sourcePosition, or reimplement
   * using
   * 
   * @see EmbeddedMontiArcASTComponentCoCo#check(ASTComponent)
   */
  @Override
  public void check(ASTComponent node) {
    EMAComponentSymbol symbol = (EMAComponentSymbol) node.getSymbolOpt().orElse(null);
    
    if (null == symbol) {
      Log.error(String.format("0x9AF6C ASTComponent node \"%s\" has no symbol. Did you forget to "
          + "run the SymbolTableCreator before checking cocos?", node.getName()));
      return;
    }
    
    for (EMAComponentInstantiationSymbol instanceSymbol : symbol.getSubComponents()) {
      for (EMAConnectorSymbol emaConnectorSymbol : instanceSymbol.getSimpleConnectors()) {
        
        EMAComponentSymbolReference typeReference = instanceSymbol.getComponentType();
        
        if (!typeReference.existsReferencedSymbol()) {
          Log.error(String.format("0xBEA8B The component type \"%s\" can't be resolved.",
              typeReference.getFullName()));
          return;
        }
        
        EMAComponentSymbol sourceComponent = typeReference.getReferencedSymbol();
        String sourcePort = emaConnectorSymbol.getSource();
        
        Optional<EMAPortSymbol> outgoingPort = sourceComponent.getOutgoingPort(sourcePort);
        
        if (!outgoingPort.isPresent()) {
          Log.error(String.format("0xF4D71 Out port \"%s\" is not present in component \"%s\".",
              sourcePort, sourceComponent.getName()),
              emaConnectorSymbol.getSourcePosition());
        }
      }
    }
  }
  
}
