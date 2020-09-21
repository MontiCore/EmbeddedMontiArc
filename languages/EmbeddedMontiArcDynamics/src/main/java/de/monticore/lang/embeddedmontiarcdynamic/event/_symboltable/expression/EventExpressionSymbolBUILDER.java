/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

import de.monticore.ast.ASTNode;
import de.monticore.commonexpressions._ast.ASTBooleanAndOpExpression;
import de.monticore.commonexpressions._ast.ASTBooleanOrOpExpression;
import de.monticore.commonexpressions._ast.ASTBracketExpression;
import de.monticore.commonexpressions._ast.ASTLogicalNotExpression;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.helper.ArcTypePrinter;
import de.monticore.lang.embeddedmontiarcdynamic.event._ast.*;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.ComponentEventSymbolReference;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValueSymbolBUILDER;
import de.monticore.lang.embeddedmontiarcdynamic.event.helper.EventTypeHelper;
import de.monticore.lang.monticar.common2._ast.ASTArrayAccess;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolutionExpression;
import de.monticore.lang.monticar.types2._ast.ASTBooleanExpression;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

public class EventExpressionSymbolBUILDER  {

    //<editor-fold desc="Single Instance">
    protected static EventExpressionSymbolBUILDER INSTANCE = null;
    protected static EventExpressionSymbolBUILDER getINSTANCE(){
        if(INSTANCE == null){
            EventExpressionSymbolBUILDER.init();
        }
        return INSTANCE;
    }
    protected void setINSTANCE(EventExpressionSymbolBUILDER builder){
        INSTANCE = builder;
    }
    public static EventExpressionSymbolBUILDER init(){
        EventExpressionSymbolBUILDER b = new EventExpressionSymbolBUILDER();
        b.setINSTANCE(b);
        return b;
    }

    //</editor-fold>

    //<editor-fold desc="Public static entry point">
    public static EventExpressionSymbol build(final ASTExpression node, final Scope definingScopeOfReference){
        return getINSTANCE().buildEventExpressionSymbol(node, definingScopeOfReference);
    }
    //</editor-fold>


    public EventExpressionSymbol buildEventExpressionSymbol(final ASTExpression node, final Scope definingScopeOfReference){
        if(node.getSymbolOpt().isPresent()){
            return (EventExpressionSymbol) node.getSymbolOpt().get();
        }

        EventExpressionSymbol ees = null;

        if(node instanceof ASTBooleanAndOpExpression) {
            ees = handle((ASTBooleanAndOpExpression) node, definingScopeOfReference);
        }else if(node instanceof ASTBooleanOrOpExpression) {
            ees = handle((ASTBooleanOrOpExpression) node, definingScopeOfReference);
        }else if(node instanceof ASTLogicalNotExpression){
            ees =  handle((ASTLogicalNotExpression)node, definingScopeOfReference);
        }else if(node instanceof ASTPortExpression) {
            ees = handle((ASTPortExpression) node);
        }else if(node instanceof ASTBooleanExpression){
            ees = new EventBooleanExpressionSymbol(((ASTBooleanExpression) node).getBooleanLiteral().getValue());
        }else if(node instanceof ASTTrueExpression){
            ees = new EventBooleanExpressionSymbol(true);
        }else if(node instanceof ASTFalseExpression){
            ees = new EventBooleanExpressionSymbol(false);
        }else if(node instanceof ASTBracketExpression){
            ees = handle((ASTBracketExpression)node, definingScopeOfReference);
        }else if(node instanceof ASTUnitNumberResolutionExpression){

            ees = buildComponentEventSymbolReference((ASTUnitNumberResolutionExpression)node, definingScopeOfReference);

        }else if(node instanceof ASTEventReferenceExpression){
            ees =buildComponentEventSymbolReference((ASTEventReferenceExpression)node, definingScopeOfReference);
        }


        return ees;
    }




    protected EventExpressionSymbol handle(ASTPortExpression node){
        EventExpressionSymbol ees = null;
        EventPortExpression ipe = null;


        ASTPortExpressionContent pec = node.getPortExpressionContent();
        if(pec instanceof ASTPortExpressionValue){
            ees = buildEventPortExpressionValueSymbol((ASTPortExpressionValue) pec);
        }else if(pec instanceof ASTPortExpressionConnect){
            ees = buildEventPortExpressionConnectSymbol((ASTPortExpressionConnect)pec);
        }else if(pec instanceof ASTPortExpressionFree){
            ees = buildEventPortExpressionFreeSymbol((ASTPortExpressionFree)pec);
        }
//        if(node.isPresentPortExpressionConnect()){
//            ees = buildEventPortExpressionConnectSymbol(node.getPortExpressionConnect());
//        }else if(node.isPresentPortExpressionFree()){
//            ees = buildEventPortExpressionFreeSymbol(node.getPortExpressionFree());
//        }else if(node.isPresentPortExpressionValue()){
//            ees = buildEventPortExpressionValueSymbol(node.getPortExpressionValue());
//        }
        //ees = buildEventPortExpressionSymbol(node.getPortExpressionContent());

        ipe = (EventPortExpression)ees;
//Todo handle array of port
        ipe.setPortName(node.getPortName().getPortName());
        if(node.getPortName().isPresentCompName()){

            ipe.setComponentName(node.getPortName().getCompName());


        }

        if(node.getPortName().isPresentPortArray()){
            ASTArrayAccess aa = node.getPortName().getPortArray();
            if(aa.isCOLON()){
                ipe.setHasPortArray(true);
            }else if(aa.isPresentIntLiteral()){
                ipe.setPortArraySingleValue(aa.getIntLiteral().getNumber().get().intValue());
            }else{
                ipe.setPortArrayLowerAndUpperBounds(aa.getLowerbound().getNumber().get().intValue(),
                        aa.getUpperbound().getNumber().get().intValue());
            }
        }

        return ees;

    }

    protected EventPortExpressionValueSymbol buildEventPortExpressionValueSymbol(ASTPortExpressionValue node){
        EventPortExpressionValueSymbol epevs = new EventPortExpressionValueSymbol();
        epevs.setPortValue(PortValueSymbolBUILDER.build(node.getPortValue()));
        return epevs;
    }

    protected EventPortExpressionConnectSymbol buildEventPortExpressionConnectSymbol(ASTPortExpressionConnect node){
        return new EventPortExpressionConnectSymbol();
    }

    protected EventPortExpressionFreeSymbol buildEventPortExpressionFreeSymbol(ASTPortExpressionFree node){
        return new EventPortExpressionFreeSymbol();
    }

    protected EventBracketExpressionSymbol handle(ASTBracketExpression node, final Scope definingScopeOfReference){
        return new EventBracketExpressionSymbol(build(node.getExpression(), definingScopeOfReference));

    }

    //<editor-fold desc="Basic Boolean Operations AST">

    protected EventExpressionSymbol handle(ASTBooleanAndOpExpression node, final Scope definingScopeOfReference ){
        return helperEventLogicalOperationExpressionSymbol(node, "&&", node.getLeftExpression(), node.getRightExpression(),definingScopeOfReference);
    }

    protected EventExpressionSymbol handle(ASTBooleanOrOpExpression node,final Scope definingScopeOfReference){
        return helperEventLogicalOperationExpressionSymbol(node, "||", node.getLeftExpression(), node.getRightExpression(), definingScopeOfReference);
    }

    protected EventExpressionSymbol handle(ASTLogicalNotExpression node,final Scope definingScopeOfReference){
        return helperEventLogicalOperationExpressionSymbol(node, "!", node.getExpression(),definingScopeOfReference);
    }

    protected EventLogicalOperationExpressionSymbol  helperEventLogicalOperationExpressionSymbol(ASTNode node, String operator, ASTExpression exp,final Scope definingScopeOfReference){
        return helperEventLogicalOperationExpressionSymbol(node, operator, null, exp, definingScopeOfReference);
    }
    protected EventLogicalOperationExpressionSymbol helperEventLogicalOperationExpressionSymbol(ASTNode node, String operator, ASTExpression left, ASTExpression right, final Scope definingScopeOfReference){
        EventExpressionSymbol l = null,r=null;
        if(left != null) {
            l = build(left, definingScopeOfReference);
        }
        if(right != null) {
            r = build(right, definingScopeOfReference);
        }
        return new EventLogicalOperationExpressionSymbol(operator, l, r);
    }

    //</editor-fold>

    //<editor-fold desc="Symbol Reference">

    protected EventReferenceExpressionSymbol buildComponentEventSymbolReference(ASTEventReferenceExpression node, final Scope definingScopeOfReference){
        String referencedCompName;

        referencedCompName = ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(node.getType());
        Log.debug(node.getType().toString(), "Type");
        String simpleCompName = Names.getSimpleName(referencedCompName);


        ComponentEventSymbolReference ref= new ComponentEventSymbolReference(referencedCompName, definingScopeOfReference);

        EventReferenceExpressionSymbol cesr = new EventReferenceExpressionSymbol(simpleCompName, ref);

        EventTypeHelper.addTypeArgumentsToEventReferenceExpression(cesr, node.getTypeOpt().orElse(null), definingScopeOfReference);

        for(ASTPortValue argument: node.getArgumentsList()){
            cesr.addArgument(PortValueSymbolBUILDER.build(argument));
        }

        EventExpressionSymbol epe = cesr.getCondition();

        return cesr;
    }

    protected EventReferenceExpressionSymbol buildComponentEventSymbolReference(ASTUnitNumberResolutionExpression node, final Scope definingScopeOfReference){
        String referencedCompName;
        ASTUnitNumberResolution innerNode = node.getUnitNumberResolution();


        if(!innerNode.isPresentName()){
            Log.error("Case not handled. Only event reference name is valid here!");
        }

        referencedCompName = innerNode.getName();//ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(node.getName());
        Log.debug(referencedCompName, "Type");
        String simpleCompName = Names.getSimpleName(referencedCompName);


        ComponentEventSymbolReference ref= new ComponentEventSymbolReference(referencedCompName, definingScopeOfReference);

        EventReferenceExpressionSymbol cesr = new EventReferenceExpressionSymbol(simpleCompName, ref);
        //EventTypeHelper.addTypeArgumentsToEventReferenceExpression(cesr, node.getTypeOpt().orElse(null), definingScopeOfReference);

//        for(ASTPortValue argument: node.getArgumentsList()){
//            cesr.addArgument(PortValueSymbolBUILDER.build(argument));
//        }

        EventExpressionSymbol epe = cesr.getCondition();

        return cesr;
    }



    //</editor-fold>
}
