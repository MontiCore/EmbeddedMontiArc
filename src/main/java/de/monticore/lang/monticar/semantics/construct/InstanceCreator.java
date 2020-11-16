/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnectorTargets;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTQualifiedNameWithArrayAndStar;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.EmbeddedMontiArcMathMill;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicPortInstanceSymbol;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.CommonMCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;

import java.util.Collection;

public class InstanceCreator {

    public static EMADynamicPortInstanceSymbol createPortInstanceSymbol(String name, String packageName, String type, boolean incoming, MutableScope enclosingScope) {
        EMADynamicPortInstanceSymbol portInstanceSymbol = new EMADynamicPortInstanceSymbol(name);
        portInstanceSymbol.setDirection(incoming);
        portInstanceSymbol.setPackageName(packageName);
        portInstanceSymbol.setFullName(packageName + "." + name);
        portInstanceSymbol.setEnclosingScope(enclosingScope);
        enclosingScope.add(portInstanceSymbol);

        CommonMCTypeReference typeReference = new CommonMCTypeReference(type, MCTypeSymbol.KIND, enclosingScope);
        portInstanceSymbol.setTypeReference(typeReference);

        // ASTNode
        ASTElementType astType = EmbeddedMontiArcMathMill.elementTypeBuilder()
                .setName(type).build();

        ASTPort astNode = EmbeddedMontiArcMathMill.portBuilder()
                .setName(name)
                .setType(astType)
                .setEnclosingScope(enclosingScope)
                .setSymbol(portInstanceSymbol)
                .build();
        portInstanceSymbol.setAstNode(astNode);

        return portInstanceSymbol;
    }

    public static EMADynamicConnectorInstanceSymbol createConnectorInstanceSymbol(String source, String target, String packageName, MutableScope enclosingScope) {
        EMADynamicConnectorInstanceSymbol connectorInstanceSymbol = new EMADynamicConnectorInstanceSymbol(target);
        connectorInstanceSymbol.setSource(source);
        connectorInstanceSymbol.setTarget(target);
        connectorInstanceSymbol.setPackageName(packageName);
        connectorInstanceSymbol.setFullName(packageName + "." + target);
        connectorInstanceSymbol.setEnclosingScope(enclosingScope);
        enclosingScope.add(connectorInstanceSymbol);

        // ASTNode
        String sourceComp = source.contains(".") ? source.substring(0, source.indexOf(".")) : null;
        String sourcePort = source.contains(".") ? source.substring(source.indexOf(".") + 1) : source;
        String targetComp = target.contains(".") ? target.substring(0, target.indexOf(".")) : null;
        String targetPort = target.contains(".") ? target.substring(target.indexOf(".") + 1) : target;

        ASTQualifiedNameWithArray sourceAstQualifiedNameWithArray = EmbeddedMontiArcMathMill.qualifiedNameWithArrayBuilder()
                .setPortName(sourcePort).setCompName(sourceComp).build();
        ASTQualifiedNameWithArrayAndStar astSource = EmbeddedMontiArcMathMill.qualifiedNameWithArrayAndStarBuilder()
                .setQualifiedNameWithArray(sourceAstQualifiedNameWithArray).build();
        ASTQualifiedNameWithArray targetAstQualifiedNameWithArray = EmbeddedMontiArcMathMill.qualifiedNameWithArrayBuilder()
                .setPortName(targetPort).setCompName(targetComp).build();
        ASTQualifiedNameWithArrayAndStar astTarget = EmbeddedMontiArcMathMill.qualifiedNameWithArrayAndStarBuilder()
                .setQualifiedNameWithArray(targetAstQualifiedNameWithArray).build();
        ASTConnectorTargets connectorTargets = EmbeddedMontiArcMathMill.connectorTargetsBuilder()
                .addQualifiedNameWithArrayAndStar(astTarget).build();

        ASTConnector astNode = EmbeddedMontiArcMathMill.connectorBuilder()
                .setSource(astSource).setTargets(connectorTargets)
                .setSymbol(connectorInstanceSymbol)
                .setEnclosingScope(enclosingScope)
                .build();
        connectorInstanceSymbol.setAstNode(astNode);

        return connectorInstanceSymbol;
    }

    public static EMAComponentInstanceSymbol rename(EMAComponentInstanceSymbol symbol, String newName) {
        EMAComponentInstanceSymbol newSymbol;
        if (symbol instanceof EMADynamicComponentInstanceSymbol) {
            newSymbol = (new EMADynamicComponentInstanceBuilder())
                    .setName(newName)
                    .setSymbolReference(symbol.getComponentType())
                    .setPackageName(symbol.getPackageName())
                    .build();
            ((EMADynamicComponentInstanceSymbol) newSymbol)
                    .setDynamicInstance(((EMADynamicComponentInstanceSymbol) symbol).isDynamicInstance());
        } else {
            newSymbol = (new EMAComponentInstanceBuilder())
                    .setName(newName)
                    .setSymbolReference(symbol.getComponentType())
                    .setPackageName(symbol.getPackageName())
                    .build();
        }
        newSymbol.setAccessModifier(symbol.getAccessModifier());
        newSymbol.setAstNode(symbol.getAstNode().orElse(null));
        newSymbol.setActualTypeArguments(symbol.getActualTypeArguments());
        newSymbol.setArguments(symbol.getArguments());
        newSymbol.setEnclosingScope(symbol.getEnclosingScope().getAsMutableScope());
        newSymbol.setFullName(symbol.getPackageName() + "." + newName);
        newSymbol.setParameters(symbol.getParameters());
        newSymbol.setResolutionDeclarationSymbols(symbol.getResolutionDeclarationSymbols());

        CommonScope spannedScope = (CommonScope) newSymbol.getSpannedScope();
        spannedScope.setResolvingFilters(symbol.getSpannedScope().getResolvingFilters());
        for (Collection<Symbol> symbols : symbol.getSpannedScope().getLocalSymbols().values()) {
            for (Symbol symbol1 : symbols) {
                spannedScope.add(symbol1);
            }
        }

        return newSymbol;
    }
}
