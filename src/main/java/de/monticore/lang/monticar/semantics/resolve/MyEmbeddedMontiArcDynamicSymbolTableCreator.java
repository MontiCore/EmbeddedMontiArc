/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponentInstance;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.EmbeddedMontiArcMathMill;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableCreator;
import de.monticore.lang.monticar.semantics.construct.ComponentReplacement;
import de.monticore.lang.monticar.semantics.construct.Replacement;
import de.monticore.lang.monticar.semantics.construct.SynthesizedComponentSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.monticore.types.types._ast.ASTImportStatement;
import de.monticore.types.types._ast.ASTReferenceType;

import java.util.*;
import java.util.stream.Collectors;

public class MyEmbeddedMontiArcDynamicSymbolTableCreator extends EmbeddedMontiArcDynamicSymbolTableCreator {

    protected static Stack<Stack<String>> instanceStack = new Stack<>();
    private Replacement replacements;

    public MyEmbeddedMontiArcDynamicSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    public MyEmbeddedMontiArcDynamicSymbolTableCreator(ResolvingConfiguration resolverConfig, MutableScope enclosingScope) {
        super(resolverConfig, enclosingScope);
    }

    @Override
    public void visit(ASTEMACompilationUnit node) {
        if (instanceStack.isEmpty()) {
            instanceStack.push(new Stack<>());
            String packageName = String.join(".", node.getPackageList());
            instanceStack.peek().push(packageName + "." + node.getComponent().getName());
        }

        List<ComponentReplacement> currentComponentReplacements = getCurrentComponentReplacements();
        for (ComponentReplacement componentReplacement : currentComponentReplacements) {
            addImportStatement(node, componentReplacement.getPackageName(), componentReplacement.getType());
            removeSubComponent(node, componentReplacement.getOldInstanceName());
            addSubComponent(node, componentReplacement.getType(), componentReplacement.getNewInstanceName());
        }

        super.visit(node);
    }

    @Override
    public void endVisit(ASTEMACompilationUnit node) {
        super.endVisit(node);

        List<ComponentReplacement> currentComponentReplacements = getCurrentComponentReplacements();
        for (ComponentReplacement componentReplacement : currentComponentReplacements) {
            replaceSymbolWithSynthesizedKind(node, componentReplacement);
        }

        instanceStack.pop();
    }

    @Override
    public void visit(ASTSubComponent node) {
        instanceStack.push(new Stack<>());

        ListIterator<ASTSubComponentInstance> listIterator =
                node.getInstancesList().listIterator(node.getInstancesList().size());
        while (listIterator.hasPrevious()) {
            ASTSubComponentInstance subcomponent = listIterator.previous();
            instanceStack.peek().push(subcomponent.getName());
        }

        super.visit(node);
    }

    private void removeSubComponent(ASTEMACompilationUnit node, String instanceNameToReplace) {
        Iterator<ASTElement> elementIterator = node.getComponent().getBody().getElementList().iterator();
        while (elementIterator.hasNext()) {
            ASTElement element = elementIterator.next();
            if (element instanceof ASTSubComponent) {
                ASTSubComponent subComponent = (ASTSubComponent) element;
                if (subComponent.getInstancesList().stream().map(s -> s.getName()).collect(Collectors.toList()).contains(instanceNameToReplace)) {
                    if (subComponent.getInstancesList().size() > 1) {
                        Iterator<ASTSubComponentInstance> iterator = subComponent.getInstancesList().iterator();
                        while (iterator.hasNext()) {
                            ASTSubComponentInstance next = iterator.next();
                            if (next.getName().equals(instanceNameToReplace))
                                iterator.remove();
                        }
                    } else {
                        elementIterator.remove();
                    }
                }
            }
        }
    }

    private void addImportStatement(ASTEMACompilationUnit node, String packageName, String type) {
        String importString = packageName.equals("") ? type :
                packageName + "." + type;
        String[] split = importString.split("\\.");
        if (split.length == 0) {
            split = new String[]{importString};
        }
        ASTImportStatement importStatement = EmbeddedMontiArcMathMill.importStatementBuilder().addAllImports(Arrays.asList(split)).build();
        node.getImportStatementList().add(importStatement);
    }

    private void addSubComponent(ASTEMACompilationUnit node, String type, String newInstanceName) {
        ASTReferenceType referenceType = EmbeddedMontiArcMathMill.simpleReferenceTypeBuilder().addName(type).build();
        ASTSubComponentInstance subComponentInstance = EmbeddedMontiArcMathMill.subComponentInstanceBuilder().setName(newInstanceName).build();
        ASTSubComponent subComponent = EmbeddedMontiArcMathMill.subComponentBuilder().setType(referenceType).addInstances(subComponentInstance).build();

        node.getComponent().getBody().addElement(subComponent);
    }

    private void replaceSymbolWithSynthesizedKind(ASTEMACompilationUnit node, ComponentReplacement componentReplacement) {
        Scope spannedScope = node.getComponent().getSpannedScope();
        EMAComponentInstantiationSymbol oldSymbol =
                spannedScope.<EMAComponentInstantiationSymbol>resolveLocally(componentReplacement.getOldInstanceName(), EMAComponentInstantiationSymbol.KIND)
                        .orElse(null);

        EMAComponentInstanceSymbol newSymbol =
                new SynthesizedComponentSymbol(oldSymbol.getName(), oldSymbol.getComponentType());

        MutableScope enclosingScope = (MutableScope) oldSymbol.getEnclosingScope();
        enclosingScope.remove(oldSymbol);
        enclosingScope.add(newSymbol);
        newSymbol.setAstNode(oldSymbol.getAstNode().orElse(null));
        newSymbol.toString();
    }

    public void setReplacements(Replacement replacements) {
        this.replacements = replacements;
    }

    private List<ComponentReplacement> getCurrentComponentReplacements() {
        String currentComponent = getCurrentComponent().toLowerCase();
        return replacements.getComponentReplacements().stream().filter(s -> s.getParentComponent().toLowerCase().equals(currentComponent)).collect(Collectors.toList());
    }

    private static String getCurrentComponent() {
        String res = instanceStack.stream().map(s -> s.peek()).collect(Collectors.joining("."));
        return res;
    }
}
