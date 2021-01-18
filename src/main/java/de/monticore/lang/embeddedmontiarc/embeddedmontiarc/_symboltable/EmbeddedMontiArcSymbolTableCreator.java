/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbolCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstancingRegister;
import de.monticore.lang.embeddedmontiarc.helper.ArcTypePrinter;
import de.monticore.lang.embeddedmontiarc.helper.EMAJavaHelper;
import de.monticore.lang.monticar.ValueSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
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

import static de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableHelper.*;
import static de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortHelper.*;
import static de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbolCreator.getGlobalScope;
import static de.monticore.lang.embeddedmontiarc.helper.EMATypeHelper.addTypeArgumentsToTypeSymbol;
import static de.monticore.lang.embeddedmontiarc.helper.EMATypeHelper.initTypeRef;
//import de.monticore.common.common._ast.ASTStereoValue;
//import de.monticore.types.TypesHelper;
//import de.monticore.types.types._ast.*;

/**
 * Visitor that creats the symboltable of an EmbeddedMontiArc AST.
 *
 */
public class EmbeddedMontiArcSymbolTableCreator extends EmbeddedMontiArcSymbolTableCreatorTOP {

    protected String compilationUnitPackage = "";

    protected EMAComponentInstanceSymbolCreator instanceSymbolCreator = new EMAComponentInstanceSymbolCreator();

    // extra stack of components that is used to determine which components are inner components.
    public Stack<EMAComponentSymbol> componentStack = new Stack<>();

    protected List<ImportStatement> currentImports = new ArrayList<>();

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

        ArtifactScope artifactScope = new EmbeddedMontiArcArtifactScope(
                Optional.empty(),
                compilationUnitPackage,
                imports);
        this.currentImports = imports;
        putOnStack(artifactScope);
    }

    public void endVisit(ASTEMACompilationUnit node) {
        // artifact scope
        removeCurrentScope();

        // creates all instances which are created through the top level component
        EMAComponentSymbol emaComponentSymbol = (EMAComponentSymbol) Log.errorIfNull(node.getComponent().getSymbolOpt().orElse(null));

        String mainComponent = InstancingRegister.mainComponent;
        String mainInstantiation = InstancingRegister.mainInstantiation;
        if(mainComponent.equals("defaultComponent")) {
            String instanceName = Character.toLowerCase(emaComponentSymbol.getName().charAt(0)) + emaComponentSymbol.getName().substring(1);
            instanceSymbolCreator.createInstances(emaComponentSymbol, instanceName);
        }

        if(mainComponent.equals(emaComponentSymbol.getFullName())) {
            instanceSymbolCreator.createInstances(emaComponentSymbol, mainInstantiation);
        }

        Log.debug("endVisit of " + node.getComponent().getSymbolOpt().get().getFullName(),
                "SymbolTableCreator:");
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
        EMAComponentSymbol component = new EMAComponentSymbol(componentName);
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
        setParametersOfComponent(component, node, this);

        // super component
        if (node.getSuperComponentOpt().isPresent()) {
            ASTReferenceType superCompRef = node.getSuperComponent();
            String superCompName = ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(superCompRef);

            EMAComponentSymbolReference ref = new EMAComponentSymbolReference(superCompName,
                    currentScope().get());
            ref.setAccessModifier(BasicAccessModifier.PUBLIC);
            // actual type arguments
            addTypeArgumentsToTypeSymbol(ref, superCompRef, this);

            component.setSuperComponent(Optional.of(ref));
        }

        // Component Modifiers
        if (!node.getComponentModifierList().isEmpty())
            component.setComponentModifiers(node.getComponentModifierList());

        // check if this component is an inner component
        if (!componentStack.isEmpty()) {
            component.setIsInnerComponent(true);
        }
        componentStack.push(component);
        addToScopeAndLinkWithNode(component, node);

        // TODO this is a hack to avoid loading one component symbol twice
        // --> must be changed in future
        Collection<Symbol> c = getGlobalScope(currentScope().get())
                .resolveDownMany(component.getFullName(), EMAComponentSymbol.KIND);
        if (c.size() > 1) {
            aboartVisitComponent = true;
            component.getEnclosingScope().getAsMutableScope()
                    .removeSubScope(component.getSpannedScope().getAsMutableScope());

            return;
        }
    }


    @Override
    public void endVisit(ASTComponent node) {
        EMAComponentSymbol component = componentStack.pop();
        removeCurrentScope();
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
        EMAPortArraySymbol pas = new EMAPortArraySymbol(name, nameTO);

        pas.setTypeReference(typeRef);
        pas.setDirection(node.isIncoming());

        addToScopeAndLinkWithNode(pas, node);
        portCreation(node, pas, name, typeRef, this);
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
        EMAComponentSymbolReference componentTypeReference = new EMAComponentSymbolReference(
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
        componentTypeReference.setArguments(node.getArgumentsList());
        componentTypeReference.fixResolutions(this);

        // PortInitials
        componentTypeReference.setPortInitials(node.getPortInitialList());


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
    public void visit(ASTConnector node) {
        doConnectorResolution(node, this);

        if (node.getSourceOpt().isPresent()) {
            if(node.getSource().isPresentDotStar()) {
                starConnectorSetup(node, this);
            } else {
                nonConstantPortSetup(node, this);
            }
        } else {
            constantPortSetup(node, this);
        }

    }


    @Override
    public void visit(ASTUnitNumberExpression node) {
        UnitNumberExpressionSymbol symbol = new UnitNumberExpressionSymbol(node);

        addToScopeAndLinkWithNode(symbol, node);
    }
}
