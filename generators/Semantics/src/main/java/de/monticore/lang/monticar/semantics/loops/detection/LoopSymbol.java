/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.Symbol;

import java.util.Collection;

public class LoopSymbol extends EMADynamicComponentInstanceSymbol {

    protected LoopSymbol(String name, EMAComponentSymbolReference type) {
        super(name, type);
    }

    public static LoopSymbol instantiate(EMAComponentInstanceSymbol symbol) {
        LoopSymbol loopSymbol = new LoopSymbol(symbol.getName(), symbol.getComponentType());
        loopSymbol.setPackageName(symbol.getPackageName());
        loopSymbol.setAccessModifier(symbol.getAccessModifier());
        loopSymbol.setAstNode(symbol.getAstNode().orElse(null));
        loopSymbol.setActualTypeArguments(symbol.getActualTypeArguments());
        loopSymbol.setArguments(symbol.getArguments());
        loopSymbol.setFullName(NameHelper.toInstanceFullQualifiedName(symbol.getPackageName(), symbol.getName()));
        loopSymbol.setParameters(symbol.getParameters());
        loopSymbol.setResolutionDeclarationSymbols(symbol.getResolutionDeclarationSymbols());

        CommonScope spannedScope = (CommonScope) loopSymbol.getSpannedScope();
        spannedScope.setResolvingFilters(symbol.getSpannedScope().getResolvingFilters());
        for (Collection<Symbol> symbols : symbol.getSpannedScope().getLocalSymbols().values()) {
            for (Symbol symbol1 : symbols) {
                spannedScope.add(symbol1);
            }
        }

        if (symbol instanceof EMADynamicComponentInstanceSymbol)
            loopSymbol.setDynamicInstance(((EMADynamicComponentInstanceSymbol) symbol).isDynamicInstance());


        return loopSymbol;
    }
}
