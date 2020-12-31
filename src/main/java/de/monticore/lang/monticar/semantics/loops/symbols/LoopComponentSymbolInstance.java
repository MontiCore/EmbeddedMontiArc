/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.construct.ConnectSourceWithTargetPort;
import de.monticore.lang.monticar.semantics.construct.InstanceCreator;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.symboltable.CommonScope;
import de.se_rwth.commons.logging.Log;

public class LoopComponentSymbolInstance extends EMADynamicComponentInstanceSymbol {

    private EMAEquationSystem eqs;

    protected LoopComponentSymbolInstance(String name, EMAComponentSymbolReference type) {
        super(name, type);
    }

    public static LoopComponentSymbolInstance instantiate(EMAComponentInstanceSymbol symbol, EMAEquationSystem eqs) {
        if (!symbol.getSubComponents().isEmpty() && !symbol.isNonVirtual()) {
            Log.error ("TODO should be atomic or nonvirtual");
        }

        LoopComponentSymbolInstance loopSymbol = new LoopComponentSymbolInstance(symbol.getName(), symbol.getComponentType());
        loopSymbol.setPackageName(symbol.getPackageName());
        loopSymbol.setAccessModifier(symbol.getAccessModifier());
        loopSymbol.setAstNode(symbol.getAstNode().orElse(null));
        loopSymbol.setActualTypeArguments(symbol.getActualTypeArguments());
        loopSymbol.setArguments(symbol.getArguments());
        loopSymbol.setFullName(NameHelper.toInstanceFullQualifiedName(symbol.getPackageName(), symbol.getName()));
        loopSymbol.setParameters(symbol.getParameters());
        loopSymbol.setResolutionDeclarationSymbols(symbol.getResolutionDeclarationSymbols());
        loopSymbol.setComponentModifiers(symbol.getComponentModifiers());

        CommonScope spannedScope = (CommonScope) loopSymbol.getSpannedScope();
        spannedScope.setResolvingFilters(symbol.getSpannedScope().getResolvingFilters());

        // Only add ports, connectors and Math Statements need to be empty, // solve equation system
        symbol.getPortInstanceList().stream()
                .filter(p -> eqs.getIncomingPorts().contains(p) || p.isOutgoing())
                .forEach(p -> {
            spannedScope.add(p);
            p.setEnclosingScope(spannedScope);
        });

        loopSymbol.eqs = eqs;

        return loopSymbol;
    }

    public void connectInformation() {
        // Connect all information (inports) from the equation system to this component
        for (EMAPortInstanceSymbol inport : eqs.getIncomingPorts()) {
            if (!getPortInstanceList().contains(inport)) {
                String newPortName = NameHelper.replaceWithUnderScore(NameHelper.calculateFullQualifiedNameOf(inport));
                EMADynamicPortInstanceSymbol portInstanceSymbol = InstanceCreator.createPortInstanceSymbol(newPortName,
                        inport.getPackageName(),
                        inport.getTypeReference().getName(),
                        true,
                        getSpannedScope().getAsMutableScope());
                getSpannedScope().getAsMutableScope().add(portInstanceSymbol);

                ConnectSourceWithTargetPort.addConnection(eqs.getAtomicSourceOf(inport).get(), portInstanceSymbol);
            }
        }
    }

    public EMAEquationSystem getEquationSystem() {
        return eqs;
    }
}
