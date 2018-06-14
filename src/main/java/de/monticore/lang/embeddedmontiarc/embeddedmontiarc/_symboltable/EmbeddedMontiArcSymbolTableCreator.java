/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.helper.ArcTypePrinter;
import de.monticore.lang.embeddedmontiarc.trafos.AutoConnection;
import de.monticore.lang.monticar.ValueSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberExpression;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.MontiCarSymbolFactory;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.*;
import de.monticore.symboltable.modifiers.BasicAccessModifier;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.TypeReference;
import de.monticore.types.types._ast.ASTImportStatement;
import de.monticore.types.types._ast.ASTReferenceType;
import de.monticore.types.types._ast.ASTType;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.StringTransformations;
import de.se_rwth.commons.logging.Log;

import java.util.*;

import static de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EMAPortHelper.*;
import static de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EMATypeHelper.addTypeArgumentsToTypeSymbol;
import static de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EMATypeHelper.initTypeRef;
import static de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcExpandedComponentInstanceSymbolCreator.getGlobalScope;
import static de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableHelper.*;
//import de.monticore.common.common._ast.ASTStereoValue;
//import de.monticore.types.TypesHelper;
//import de.monticore.types.types._ast.*;

/**
 * Visitor that creats the symboltable of an EmbeddedMontiArc AST.
 *
 * @author Sascha Schneiders, Sining Wang, Yu Qiao
 */
public class EmbeddedMontiArcSymbolTableCreator extends EmbeddedMontiArcSymbolTableCreatorTOP {

    protected String compilationUnitPackage = "";

    protected EmbeddedMontiArcExpandedComponentInstanceSymbolCreator instanceSymbolCreator = new EmbeddedMontiArcExpandedComponentInstanceSymbolCreator();

    // extra stack of components that is used to determine which components are inner components.
    protected Stack<ComponentSymbol> componentStack = new Stack<>();

    protected List<ImportStatement> currentImports = new ArrayList<>();

    protected AutoConnection autoConnectionTrafo = new AutoConnection();

    protected MontiCarSymbolFactory jSymbolFactory = new MontiCarSymbolFactory();

    protected boolean aboartVisitComponent = false;

    protected boolean autoInstantiate = false;


    public EmbeddedMontiArcSymbolTableCreator(
            final ResolvingConfiguration resolverConfig,
            final MutableScope enclosingScope) {
        super(resolverConfig, enclosingScope);
    }

    public EmbeddedMontiArcSymbolTableCreator(
            final ResolvingConfiguration resolvingConfig,
            final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    @Override
    public void visit(ASTEMACompilationUnit compilationUnit) {
        Log.debug("Building Symboltable for Component: " + compilationUnit.getComponent().getName(),
                EmbeddedMontiArcSymbolTableCreator.class.getSimpleName());
        compilationUnitPackage = Names.getQualifiedName(compilationUnit.getPackageList());

        // imports
        List<ImportStatement> imports = new ArrayList<>();
        for (ASTImportStatement astImportStatement : compilationUnit.getImportStatementList()) {
            String qualifiedImport = Names.getQualifiedName(astImportStatement.getImportList());
            ImportStatement importStatement = new ImportStatement(qualifiedImport,
                    astImportStatement.isStar());
            imports.add(importStatement);
        }
        EMAJavaHelper.addJavaDefaultImports(imports);

        ArtifactScope artifactScope = new EmbeddedMontiArcArtifactScope(
                Optional.empty(),
                compilationUnitPackage,
                imports);
        this.currentImports = imports;
        putOnStack(artifactScope);
    }

    public void endVisit(ASTEMACompilationUnit node) {
        // TODO clean up component types from references to inner components
        // cleanUpReferences();

        // artifact scope
        removeCurrentScope();

        if (aboartVisitComponent) {
            return;
        }
        // creates all instances which are created through the top level component
        Log.debug("endVisit of " + node.getComponent().getSymbolOpt().get().getFullName(),
                "SymbolTableCreator:"); // ,"MontiArcSymbolTableCreator");
        // new Error().printStackTrace();
        instanceSymbolCreator.createInstances(
                (ComponentSymbol) (Log.errorIfNull(node.getComponent().getSymbolOpt().orElse(null))));
    }


    @Override
    public void visit(ASTUnitNumberExpression node) {
        UnitNumberExpressionSymbol symbol = new UnitNumberExpressionSymbol(node);

        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void visit(ASTPort node) {

        String nameTO = doPortResolution(node, this);
        ASTType astType = node.getType();
        if (node.getType() instanceof ASTCommonMatrixType) {
            getRealThis().handle((ASTCommonMatrixType) node.getType());
        }
        StringBuilder typeName = new StringBuilder();
        MCTypeReference<? extends MCTypeSymbol> typeRef = initTypeRef(node, typeName, astType, this);
        String name = node.getNameOpt().orElse(StringTransformations.uncapitalize(typeName.toString()));
        /* Log.debug(nameTO, "NameResolution:"); Log.debug(name, "Full Name:");
         * Log.debug(node.getType().toString(), "Node:"); Log.debug("" +
         * currentScope().get().toString(), "Scope:"); */
        PortArraySymbol pas = new PortArraySymbol(name, nameTO);

        pas.setTypeReference(typeRef);
        pas.setDirection(node.isIncoming());

        addToScopeAndLinkWithNode(pas, node);

        portCreation(node, pas, name, typeRef, this);
    }


    @Override
    public void visit(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector node) {
        doConnectorResolution(node, this);
        ASTQualifiedNameWithArray portName;
        List<String> sourceNames = null;
        boolean isConstant = false;
        if (node.getSourceOpt().isPresent()) {
            portName = node.getSource();

            sourceNames = getPortName(portName, this);
            // Log.debug(node.getSource().toString(),"port content");
        } else {
            isConstant = true;
            // Log.debug(node.getSI_Unit().get().toString(), "port content else ");
            constantPortSetup(node, this);
        }
        if (!isConstant) {
            nonConstantPortSetup(sourceNames, node, this);
        }

    }

    @Override
    public void visit(ASTMontiArcAutoInstantiate node) {
        autoInstantiate = node.isOn();
    }


    @Override
    public void visit(ASTSubComponent node) {
        String referencedCompName;
        /* if (node.getType() instanceof ASTSimpleReferenceType) referencedCompName =
         * ArcTypePrinter.printSubComponentName(node); else */
        referencedCompName = ArcTypePrinter
                .printTypeWithoutTypeArgumentsAndDimension(node.getType());
        Log.debug(node.getType().toString(), "Type");
        // String refCompPackage = Names.getQualifier(referencedCompName);
        String simpleCompName = Names.getSimpleName(referencedCompName);
        Log.debug(referencedCompName, "referencedCompName");
        Log.debug(currentScope().get().toString(), "Scope");
        ComponentSymbolReference componentTypeReference = new ComponentSymbolReference(
                referencedCompName,
                currentScope().get(), this);

        // set actual Resolution values
        setActualResolutionDeclaration(node, componentTypeReference);

        // actual type arguments
        // TODO enable if needed
        addTypeArgumentsToTypeSymbol(componentTypeReference, node.getType(), this);

        // ref.setPackageName(refCompPackage);

        // TODO internal representation of ValueSymbol ? that was heavily based on CommonValues
        // language and its expressions, but we use JavaDSL.
        List<ValueSymbol<TypeReference<TypeSymbol>>> configArgs = new ArrayList<>();
        /* for (ASTExpression arg : node.getArguments()) { String value = new JavaDSLPrettyPrinter(new
         * IndentPrinter()).prettyprint(arg); value = value.replace("\"", "\\\"").replace("\n", "");
         * configArgs.add(new ValueSymbol<>(value, Kind.Expression)); } */
        for (ASTExpression astExpression : node.getArgumentsList())
            componentTypeReference.addArgument(astExpression);
        componentTypeReference.fixResolutions(this);
        // instances

        if (!node.getInstancesList().isEmpty()) {
            // create instances of the referenced components.
            for (ASTSubComponentInstance i : node.getInstancesList()) {
                // For generic type resolution Example: <N1 n=4> with instance being <6> to change value of
                // n accordingly
                doSubComponentInstanceResolution(i, componentTypeReference, this);
                Log.debug(node.getType().toString(), "Pre Handle Size:");

                if (i.getUnitNumberResolutionOpt().isPresent()) {
                    int size = i.getUnitNumberResolution().getNumber().get().intValue();

                    Log.debug(node.getType().toString(), "First: ");
                    Log.debug(node.getType().toString(), "Second: ");

                    for (int ii = 1; ii <= size; ++ii) {
                        createInstance(i.getName() + "[" + ii + "]", node, componentTypeReference, configArgs, this);
                    }
                } else {
                    createInstance(i.getName(), node, componentTypeReference, configArgs, this);
                }
            }
        } else {
            // auto instance because instance name is missing
            createInstance(StringTransformations.uncapitalize(simpleCompName), node,
                    componentTypeReference, new ArrayList<>(), this);
        }

        node.setEnclosingScope(currentScope().get());
    }


    @Override
    public void handle(ASTComponent node) {
        getRealThis().visit(node);
        if (!aboartVisitComponent) {
            getRealThis().traverse(node);
            getRealThis().endVisit(node);
        }
    }


    @Override
    public void visit(ASTComponent node) {
        String componentName = node.getName();

        String componentPackageName = "";
        if (componentStack.isEmpty()) {
            // root component (most outer component of the diagram)
            componentPackageName = compilationUnitPackage;
        } else {
            // inner component uses its parents component full name as package
            componentPackageName = componentStack.peek().getFullName();
        }
        ComponentSymbol component = new ComponentSymbol(componentName);
        component.setImports(currentImports);
        component.setPackageName(componentPackageName);

        // Handle ResolutionDeclaration of stuff like <N1 n=5>
        if (node.getGenericTypeParametersOpt().isPresent()) {
            handleResolutionDeclaration(component, node.getGenericTypeParametersOpt().get(), currentScope().get(),
                    node, this);
        }

        Log.debug(component.toString(), "ComponentPreGeneric");
        // generic type parameters
        if (node.getGenericTypeParametersOpt().isPresent()) {
            EMAJavaHelper.addTypeParametersToType(component, node.getGenericTypeParametersOpt().get(),
                    currentScope().get());
        }

        Log.debug(component.toString(), "ComponentPostGeneric");
        // parameters
        // Log.debug(node.getHead().toString(),"ASTComponentHead");
        setParametersOfComponent(component, node, this);
        // Log.debug(component.toString(),"ComponentPostParam");

        // super component
        if (node.getSuperComponentOpt().isPresent()) {
            ASTReferenceType superCompRef = node.getSuperComponent();
            String superCompName = ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(superCompRef);

            ComponentSymbolReference ref = new ComponentSymbolReference(superCompName,
                    currentScope().get());
            ref.setAccessModifier(BasicAccessModifier.PUBLIC);
            // actual type arguments
            addTypeArgumentsToTypeSymbol(ref, superCompRef, this);

            component.setSuperComponent(Optional.of(ref));
        }

        // check if this component is an inner component
        if (!componentStack.isEmpty()) {
            component.setIsInnerComponent(true);
        }

        componentStack.push(component);

        addToScopeAndLinkWithNode(component, node);

        // TODO this is a hack to avoid loading one component symbol twice
        // --> must be changed in future
        Collection<Symbol> c = getGlobalScope(currentScope().get())
                .resolveDownMany(component.getFullName(), ComponentSymbol.KIND);
        if (c.size() > 1) {
            aboartVisitComponent = true;
            component.getEnclosingScope().getAsMutableScope()
                    .removeSubScope(component.getSpannedScope().getAsMutableScope());

            return;
        }

        autoConnectionTrafo.transformAtStart(node, component);
    }

    @Override
    public void visit(ASTMontiArcAutoConnect node) {
        autoConnectionTrafo.transform(node, componentStack.peek());
    }

    @Override
    public void endVisit(ASTComponent node) {
        ComponentSymbol component = componentStack.pop();
        autoConnectionTrafo.transformAtEnd(node, component);

        removeCurrentScope();

    }

    public void removeFromScope(Symbol symbol) {
        currentScope().get().remove(symbol);
    }

    public MutableScope getCurrentScopeAsMutableScope() {
        return currentScope().get();
    }
}
