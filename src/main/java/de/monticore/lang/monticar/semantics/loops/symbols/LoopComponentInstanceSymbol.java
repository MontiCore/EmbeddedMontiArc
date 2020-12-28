/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.Symbol;

import java.util.Collection;

public class LoopComponentInstanceSymbol extends EMADynamicComponentInstanceSymbol {

    protected LoopComponentInstanceSymbol(String name, EMAComponentSymbolReference type) {
        super(name, type);
    }

    public static LoopComponentInstanceSymbol instantiate(EMAComponentInstanceSymbol symbol) {
        LoopComponentInstanceSymbol loopComponent = new LoopComponentInstanceSymbol(symbol.getName(), symbol.getComponentType());
        loopComponent.setPackageName(symbol.getPackageName());
        loopComponent.setAccessModifier(symbol.getAccessModifier());
        loopComponent.setAstNode(symbol.getAstNode().orElse(null));
        loopComponent.setActualTypeArguments(symbol.getActualTypeArguments());
        loopComponent.setArguments(symbol.getArguments());
        loopComponent.setFullName(NameHelper.toInstanceFullQualifiedName(symbol.getPackageName(), symbol.getName()));
        loopComponent.setParameters(symbol.getParameters());
        loopComponent.setResolutionDeclarationSymbols(symbol.getResolutionDeclarationSymbols());
        loopComponent.setComponentModifiers(symbol.getComponentModifiers());

        CommonScope spannedScope = (CommonScope) loopComponent.getSpannedScope();
        spannedScope.setResolvingFilters(symbol.getSpannedScope().getResolvingFilters());
        for (Collection<Symbol> symbols : symbol.getSpannedScope().getLocalSymbols().values()) {
            for (Symbol symbol1 : symbols) {
                spannedScope.add(symbol1);
            }
        }

        return loopComponent;
    }
}
