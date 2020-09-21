/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable;

import de.monticore.ast.ASTNode;
import de.monticore.commonexpressions._ast.*;
import de.monticore.commonexpressions._visitor.CommonExpressionsVisitor;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarcdynamic.event._ast.*;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.*;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.*;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.helper.ComponentEventSymbolHelper;
import de.monticore.lang.embeddedmontiarcdynamic.event._visitor.EventDelegatorVisitor;
import de.monticore.lang.embeddedmontiarcdynamic.event._visitor.EventVisitor;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Deque;
import java.util.Optional;

public class EventSymbolTableCreator extends EventSymbolTableCreatorTOP implements EventVisitor {

    public EventSymbolTableCreator(
            final ResolvingConfiguration resolvingConfig, final MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public EventSymbolTableCreator(final ResolvingConfiguration resolvingConfig, final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);

    }


    @Override
    public void visit(ASTEventCompilationUnit astnode) {
        Log.debug("Building Symboltable for Event: " + astnode.getComponentEvent().getName(),
                EventSymbolTableCreator.class.getSimpleName());
        String compilationUnitPackage = Names.getQualifiedName(astnode.getPackageList());
        ArtifactScope artifactScope = new ArtifactScope(
                Optional.empty(),
                compilationUnitPackage,
                new ArrayList<>());
        this.putOnStack(artifactScope);
    }

    @Override
    public void endVisit(ASTEventCompilationUnit node) {
        removeCurrentScope();
    }

    @Override
    public void visit(ASTComponentEvent node) {
        ComponentEventSymbol eventSymbol = new ComponentEventSymbol(node.getName());

        Log.debug(eventSymbol.getName(), "Event Pre Generic");
        if(node.getGenericTypeParametersOpt().isPresent()){
            ComponentEventSymbolHelper.getINSTANCE().addTypeParametersToType(eventSymbol, node.getGenericTypeParameters(), currentScope().get());
        }
        Log.debug(eventSymbol.getName(), "Event Post Generic");


        Log.debug(eventSymbol.toString(), "Event Pre Param");
        ComponentEventSymbolHelper.getINSTANCE().setParametersOfEvent(eventSymbol, node, this);
        Log.debug(eventSymbol.toString(), "Event Post Param");

        node.setSymbol(eventSymbol);

        addToScopeAndLinkWithNode(eventSymbol, node);
    }

    @Override
    public void endVisit(ASTComponentEvent node) {
//        EventExpressionSymbol o = (EventExpressionSymbol) node.getCondition().getSymbolOpt().get();
//        String s = o.getTextualRepresentation();
//        System.out.println(s);

        EventExpressionSymbol o = EventExpressionSymbolBUILDER.build(node.getCondition(), currentScope().get());

        ComponentEventSymbol ces = (ComponentEventSymbol)node.getSymbolOpt().get();
        ces.setCondition(o);

        removeCurrentScope();
    }

    @Deprecated
    @Override
    public void endVisit(ASTExpression node) {
        //node.setSymbol(node.getExpression().getSymbolOpt().get());
        EventExpressionSymbol ees = EventExpressionSymbolBUILDER.build(node, currentScope().get());

        //node.setSymbol(ees);
        addToScopeAndLinkWithNode(ees, node);
    }

//    @Override
//    public void visit(ASTEventReferenceExpression node) {
//        System.out.println(node);
//    }
//
//
//    @Override
//    public void visit(ASTPortResolutionDeclaration node) {
//        System.out.println(node);
//    }
}
