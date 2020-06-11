/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponentInstance;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.EmbeddedMontiArcMathMill;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicSymbolTableCreator;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.monticore.types.types._ast.ASTImportStatement;
import de.monticore.types.types._ast.ASTReferenceType;

import java.util.*;
import java.util.stream.Collectors;

public class MyEmbeddedMontiArcDynamicSymbolTableCreator extends EmbeddedMontiArcDynamicSymbolTableCreator {

    protected static Stack<Stack<String>> instanceStack = new Stack<>();
    private Set<ComponentReplacement> componentReplacements;

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
            instanceStack.peek().push(node.getComponent().getName());
        }

        List<ComponentReplacement> currentComponentReplacements = getCurrentReplacements();
        for (ComponentReplacement componentReplacement : currentComponentReplacements) {
            addImportStatement(node, componentReplacement.getPackageName(), componentReplacement.getType());
            removeSubComponent(node, componentReplacement.getInstanceNameToReplace());
            addSubComponent(node, componentReplacement.getType(), componentReplacement.getNewInstanceName());
        }

        super.visit(node);
    }

    @Override
    public void endVisit(ASTEMACompilationUnit node) {
        super.visit(node);

        List<ComponentReplacement> currentComponentReplacements = getCurrentReplacements();
        for (ComponentReplacement componentReplacement : currentComponentReplacements) {
            replaceSymbolWithSynthesizedKind(node, componentReplacement);
        }

        instanceStack.pop();
    }

    @Override
    public void visit(ASTSubComponent node) {
        ListIterator<ASTSubComponentInstance> listIterator = node.getInstancesList().listIterator();
        instanceStack.push(new Stack<>());

        while (listIterator.hasNext())
            listIterator.next();
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
        EMADynamicComponentInstantiationSymbol oldSymbol =
                spannedScope.<EMADynamicComponentInstantiationSymbol>resolve(componentReplacement.getNewInstanceName(), EMADynamicComponentInstantiationSymbol.KIND)
                        .orElse(null);

        EMADynamicComponentInstantiationSymbol newSymbol =
                new SynthesizedComponentSymbol(oldSymbol.getName(), oldSymbol.getDynamicComponentType());

        MutableScope enclosingScope = (MutableScope) oldSymbol.getEnclosingScope();
        enclosingScope.remove(oldSymbol);
        enclosingScope.add(newSymbol);
        newSymbol.setAstNode(oldSymbol.getAstNode().orElse(null));
        newSymbol.toString();
    }

    public void setComponentReplacements(Set<ComponentReplacement> componentReplacements) {
        this.componentReplacements = componentReplacements;
    }

    private List<ComponentReplacement> getCurrentReplacements() {
        String currentComponent = getCurrentComponent();
        return componentReplacements.stream().filter(s -> s.getParentComponent().equals(currentComponent)).collect(Collectors.toList());
    }

    private static String getCurrentComponent() {
        String res = instanceStack.stream().map(s -> s.peek()).collect(Collectors.joining("."));
        return res;
    }
}
