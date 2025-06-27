/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.EmbeddedMontiArcMathMill;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicPortInstanceSymbol;
import de.monticore.lang.math._ast.ASTMathStatements;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.semantics.construct.ConnectSourceWithTargetPort;
import de.monticore.lang.monticar.semantics.construct.InstanceCreator;
import de.monticore.lang.monticar.semantics.helper.EMAPropertiesHelper;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.symboltable.CommonScope;
import de.se_rwth.commons.logging.Log;

import java.util.LinkedList;

public class LoopComponentInstanceSymbol extends EMADynamicComponentInstanceSymbol {

    private EMAEquationSystem eqs;

    protected LoopComponentInstanceSymbol(String name, EMAComponentSymbolReference type) {
        super(name, type);
    }

    public static LoopComponentInstanceSymbol instantiate(EMAComponentInstanceSymbol symbol, EMAEquationSystem eqs) {
        if (!EMAPropertiesHelper.isAtomic(symbol) && !EMAPropertiesHelper.isNonVirtual(symbol)) {
            Log.error ("0xEMAES4481 should be atomic or nonvirtual");
        }

        LoopComponentInstanceSymbol loopSymbol = new LoopComponentInstanceSymbol(symbol.getName(), symbol.getComponentType());
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

        if (!spannedScope.resolve("MathStatements", MathStatementsSymbol.KIND).isPresent())
            spannedScope.add(new MathStatementsSymbol("MathStatements", new ASTMathStatements()));

        return loopSymbol;
    }

    public void connectInformation() {
        // Connect all information (inports) from the equation system to this component
        for (EMAPortInstanceSymbol inport : eqs.getIncomingPorts()) {
            if (!getPortInstanceList().contains(inport)) {
                String newPortName = NameHelper.replaceWithUnderScore(NameHelper.calculateFullQualifiedNameOf(inport));
                EMADynamicPortInstanceSymbol portInstanceSymbol = InstanceCreator.createPortInstanceSymbol(newPortName,
                        getFullName(),
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
