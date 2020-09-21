/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable;


import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbolCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstancingRegister;
import de.monticore.lang.embeddedmontiarc.helper.ArcTypePrinter;
import de.monticore.lang.embeddedmontiarc.helper.EMAJavaHelper;
import de.monticore.lang.embeddedmontiarc.helper.EMATypeHelper;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTComponent;

import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTEventHandler;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTPort;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.*;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbolCreator;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._visitor.EmbeddedMontiArcDynamicDelegatorVisitor;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._visitor.EmbeddedMontiArcDynamicInheritanceVisitor;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._visitor.EmbeddedMontiArcDynamicVisitor;
//import de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTAndEventConditionExpression;
//import de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTEventConditionExpression;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventSymbolTableCreator;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventExpressionSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventExpressionSymbolBUILDER;
import de.monticore.lang.monticar.ValueSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.modifiers.BasicAccessModifier;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.TypeReference;
import de.monticore.types.types._ast.ASTReferenceType;
import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._ast.ASTType;
import de.monticore.types.types._ast.ASTTypeArgument;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.StringTransformations;
import de.se_rwth.commons.logging.Log;

import java.util.*;

import static de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableHelper.handleResolutionDeclaration;
import static de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableHelper.setParametersOfComponent;
import static de.monticore.lang.embeddedmontiarc.helper.EMATypeHelper.addTypeArgumentsToTypeSymbol;
import static de.monticore.lang.embeddedmontiarc.tagging.adaptable.AdaptableSymbolCreator.getGlobalScope;


public class EmbeddedMontiArcDynamicSymbolTableCreator extends EmbeddedMontiArcSymbolTableCreator implements EmbeddedMontiArcDynamicVisitor {

//    protected EmbeddedMontiArcDynamicDelegatorVisitor visitor;
//
//
//    protected EventSymbolTableCreator eventSTC;

    public EmbeddedMontiArcDynamicSymbolTableCreator(final ResolvingConfiguration resolvingConfig, final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
        this.innerConstructor();
    }


    public EmbeddedMontiArcDynamicSymbolTableCreator(ResolvingConfiguration resolverConfig, MutableScope enclosingScope) {
        super(resolverConfig, enclosingScope);

        this.innerConstructor();

//        visitor = new EmbeddedMontiArcDynamicDelegatorVisitor();
//
//        eventSTC = new EventSymbolTableCreator(resolverConfig, enclosingScope);
//
//        visitor.setEmbeddedMontiArcDynamicVisitor(this);
//        visitor.setEmbeddedMontiArcVisitor(this);
//        visitor.setEventVisitor(eventSTC);
    }

    protected void innerConstructor(){
        this.instanceSymbolCreator = new EMADynamicComponentInstanceSymbolCreator();
    }


    @Override
    public void endVisit(ASTEMACompilationUnit node) {
        super.endVisit(node);
    }


//<editor-fold desc="Component">

    @Override
    public void handle(ASTComponent node) {
        getRealThis().visit(node);
        if(!aboartVisitComponent)
        {
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
        EMADynamicComponentSymbol component = new EMADynamicComponentSymbol(componentName);
        component.setImports(currentImports);
        component.setPackageName(componentPackageName);
        //set is dynamic
        component.setDynamic(node.isDynamic());

        // Handle ResolutionDeclaration of stuff like <N1 n=5>
        if (node.getGenericTypeParametersOpt().isPresent()) {
            handleResolutionDeclaration(component, node.getGenericTypeParametersOpt().get(), currentScope().get(), node, this);
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

        // check if this component is an inner component
        if (!componentStack.isEmpty()) {
            component.setIsInnerComponent(true);
        }



        componentStack.push(component);
        addToScopeAndLinkWithNode(component, node);



        // TODO this is a hack to avoid loading one component symbol twice
        // --> must be changed in future
        Collection<Symbol> c = getGlobalScope(currentScope().get())
                .resolveDownMany(component.getFullName(), EMADynamicComponentSymbol.KIND);
        if (c.size() > 1) {
            aboartVisitComponent = true;
            component.getEnclosingScope().getAsMutableScope()
                    .removeSubScope(component.getSpannedScope().getAsMutableScope());

            return;
        }
    }


    @Override
    public void endVisit(ASTComponent node) {
        componentStack.pop();
        EMADynamicComponentSymbol component = (EMADynamicComponentSymbol)node.getSymbolOpt().get();
        removeCurrentScope();
    }

//</editor-fold>

//<editor-fold desc="Event Hander">
    @Override
    public void visit(ASTEventHandler node) {
        EMADynamicEventHandlerSymbol des = new EMADynamicEventHandlerSymbol();
        addToScopeAndLinkWithNode(des, node);
    }

    @Override
    public void traverse(de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTEventHandler node) {

//        if (null != node.getExpression()) {
//            node.getExpression().accept(getRealThis());
//        }
//        {
            Iterator<de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement> iter_bodys = node.getBodyList().iterator();
            while (iter_bodys.hasNext()) {
                iter_bodys.next().accept(getRealThis());
            }
//        }
    }

    @Override
    public void endVisit(ASTEventHandler node) {
        removeCurrentScope();
        EMADynamicEventHandlerSymbol des = (EMADynamicEventHandlerSymbol)node.getSymbolOpt().get();

        EventExpressionSymbol ees = null;
        if(node.getExpressionOpt().isPresent()){
            ees = EventExpressionSymbolBUILDER.build(node.getExpression(), currentScope().get());
        }

        if(node.getEventReferenceExpressionOpt().isPresent()){
            ees = EventExpressionSymbolBUILDER.build(node.getEventReferenceExpression(), currentScope().get());
        }

        des.setConditionWithSymbol(ees);
        //System.out.println(node.toString());
    }

//</editor-fold>

    @Override
    public void visit(ASTPort node) {

        EMAPortSymbol portSymbol = null;

        ASTType astType = node.getType();
        if (node.getType() instanceof ASTCommonMatrixType) {
            getRealThis().handle((ASTCommonMatrixType) node.getType());
        }
        StringBuilder typeName = new StringBuilder();
        MCTypeReference<? extends MCTypeSymbol> typeRef = EMATypeHelper.initTypeRef(node, typeName, astType, this);
        String name = node.getNameOpt().orElse(StringTransformations.uncapitalize(typeName.toString()));

        if(node.getUnitNumberResolutionOpt().isPresent()){
            EMAPortArraySymbol arraySymbol = null;

            if(node.getDynamicNumberOfPortsOpt().isPresent()){
                // dynamic port (array)
                portSymbol = EMADynamicPortHelper.getINSTANCE().newEMADynamicPortArraySymbol(node, name, typeRef, this);
            }else{
                // port array
                portSymbol = EMADynamicPortHelper.getINSTANCE().newEMAPortArraySymbol(node, name, typeRef, this);
            }
        }else {
            //normal port
            portSymbol = EMADynamicPortHelper.getINSTANCE().newEMAPortSymbol(node, name, typeRef);
        }
        addToScopeAndLinkWithNode(portSymbol, node);

    }

    @Override
    public void visit(ASTConnector node) {
        EMADynamicPortHelper.doConnectorResolution(node, this);

        if (node.getSourceOpt().isPresent()) {
            if(node.getSource().isPresentDotStar()) {
                Log.error("TODO: star connector setup");
                EMADynamicPortHelper.starConnectorSetup(node, this);
            } else {
                EMADynamicPortHelper.getINSTANCE().nonConstantPortSetup(node, this);
            }
        } else {
            EMADynamicPortHelper.getINSTANCE().constantPortSetup(node, this);
        }

    }

//    @Override
//    public void endVisit(ASTConnector node) {
//        System.out.println(node);
//    }

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

//        EMAComponentSymbolReference componentTypeReference = new EMAComponentSymbolReference(
//                referencedCompName,
//                currentScope().get(), this);

        EMADynamicComponentSymbolReference componentTypeReference = new EMADynamicComponentSymbolReference(referencedCompName, currentScope().get(), this);

//        System.out.println(componentTypeReference.getReferencedSymbol());


        // set actual Resolution values
        EmbeddedMontiArcDynamicSymbolTableHelper.setActualResolutionDeclaration(node, componentTypeReference, this);


        // actual type arguments
        // TODO enable if needed
        EMATypeHelper.addTypeArgumentsToTypeSymbol(componentTypeReference, node.getType(), this); // <- deprecated!
//        if(node.getType() instanceof ASTSimpleReferenceType){
//            ASTSimpleReferenceType simple = (ASTSimpleReferenceType)node.getType();
//            EMATypeHelper.setActualTypeArguments(componentTypeReference, simple.getTypeArguments().getTypeArgumentList(),this);
//        }

        // ref.setPackageName(refCompPackage);

        // TODO internal representation of ValueSymbol ? that was heavily based on CommonValues
        // language and its expressions, but we use JavaDSL.
        List<ValueSymbol<TypeReference<TypeSymbol>>> configArgs = new ArrayList<>();
        for (ASTExpression astExpression : node.getArgumentsList())
            componentTypeReference.addArgument(astExpression);
        componentTypeReference.fixResolutions(this);


        if (!node.getInstancesList().isEmpty()) {
            // create instances of the referenced components.
            for (de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponentInstance i : node.getInstancesList()) {
                de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTSubComponentInstance dynInstance = null;

                if(i instanceof de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTSubComponentInstance){
                    dynInstance = (de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._ast.ASTSubComponentInstance)i;
                }

                EmbeddedMontiArcSymbolTableHelper.doSubComponentInstanceResolution(i, componentTypeReference, this);

                Log.debug(node.getType().toString(), "Pre Handle Size:");
                EmbeddedMontiArcDynamicSymbolTableHelper.createInstance(i.getName(), node, componentTypeReference, configArgs, this, dynInstance);
            }
        }

        EmbeddedMontiArcDynamicSymbolTableHelper.fixResolutionsInSubComponents(this, componentTypeReference);

        node.setEnclosingScope(currentScope().get());


    }

//<editor-fold desc="(real) this">

    private EmbeddedMontiArcDynamicVisitor realThis = this;

    @Override
    public EmbeddedMontiArcDynamicVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void setRealThis(EmbeddedMontiArcDynamicVisitor realThis) {
        if(this.realThis != realThis) {
            this.realThis = realThis;
        }
    }

//</editor-fold>


    public void setInstanceSymbolCreator(EMADynamicComponentInstanceSymbolCreator symbolCreator){
        this.instanceSymbolCreator = symbolCreator;
    }
}
