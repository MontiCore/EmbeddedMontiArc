/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.builder;

import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicEventHandlerSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicPortArraySymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicEventHandlerInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.*;
import de.monticore.symboltable.MutableScope;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

public class EMADynamicEventHandlerInstanceBuilder {

    //<editor-fold desc="Single Instance">

    protected static EMADynamicEventHandlerInstanceBuilder INSTANCE = null;
    public static EMADynamicEventHandlerInstanceBuilder getINSTANCE(){
        if(INSTANCE == null){
            EMADynamicEventHandlerInstanceBuilder.init();
        }
        return INSTANCE;
    }
    protected void setINSTANCE(EMADynamicEventHandlerInstanceBuilder builder){
        INSTANCE = builder;
    }
    public static EMADynamicEventHandlerInstanceBuilder init(){
        EMADynamicEventHandlerInstanceBuilder b = new EMADynamicEventHandlerInstanceBuilder();
        b.setINSTANCE(b);
        return b;
    }

    //</editor-fold>

    public EMADynamicEventHandlerInstanceSymbol build(EMADynamicEventHandlerSymbol emaDynamicEventHandlerSymbol){
        EMADynamicEventHandlerInstanceSymbol sym = new EMADynamicEventHandlerInstanceSymbol(emaDynamicEventHandlerSymbol.getName());
        final MutableScope scope = (MutableScope) sym.getSpannedScope();

        emaDynamicEventHandlerSymbol.getSpannedScope().getResolvingFilters().stream().forEachOrdered(f -> scope.addResolver(f));

        EventExpressionSymbol ees = emaDynamicEventHandlerSymbol.getCondition().expand();

        sym.setCondition(expandExpression(emaDynamicEventHandlerSymbol, ees));


        emaDynamicEventHandlerSymbol.getConnectors().stream().forEach(obj -> {
            scope.add(EMADynamicConnectorInstanceSymbol.newAndInstantiate(obj, sym.getFullName()));
        });

        return sym;

    }

//<editor-fold desc="Expressions">
    protected EventExpressionSymbol expandExpression(EMADynamicEventHandlerSymbol emaDynamicEventHandlerSymbol, EventExpressionSymbol expressionSymbol){
        if(expressionSymbol instanceof EventBracketExpressionSymbol){
//            expandExpression(((EventBracketExpressionSymbol)expressionSymbol).getInnerExpression());
            return new EventBracketExpressionSymbol(expandExpression(emaDynamicEventHandlerSymbol,((EventBracketExpressionSymbol)expressionSymbol).getInnerExpression()));
        }else if(expressionSymbol instanceof EventLogicalOperationExpressionSymbol){
            return new EventLogicalOperationExpressionSymbol(((EventLogicalOperationExpressionSymbol) expressionSymbol).getOperator(),
                    expandExpression(emaDynamicEventHandlerSymbol, ((EventLogicalOperationExpressionSymbol) expressionSymbol).getLeftExpression()),
                    expandExpression(emaDynamicEventHandlerSymbol, ((EventLogicalOperationExpressionSymbol) expressionSymbol).getRightExpression()));
        }else if(expressionSymbol instanceof EventPortExpressionConnectSymbol){
            return expandConnectExpression(emaDynamicEventHandlerSymbol, (EventPortExpressionConnectSymbol) expressionSymbol);
        }else if(expressionSymbol instanceof EventPortExpressionValueSymbol){
            return expandValueExpression(emaDynamicEventHandlerSymbol, (EventPortExpressionValueSymbol) expressionSymbol);
        }
        return expressionSymbol;
    }

    protected EventExpressionSymbol expandConnectExpression(EMADynamicEventHandlerSymbol dehs, EventPortExpressionConnectSymbol oldSym){

        //TODO:
//        if(oldSym.hasComponentName()){
//            Collection<EMADynamicComponentInstantiationSymbol> sims = dehs.getEnclosingScope().resolveLocally(EMADynamicComponentInstantiationSymbol.KIND);
//            Log.error("Not supported atm.");
//        }else{
//            Optional<EMADynamicPortSymbol> optPort = dehs.getEnclosingScope().resolveLocally(oldSym.getPortName(), EMADynamicPortSymbol.KIND);
//            Optional<EMADynamicPortArraySymbol> opt = dehs.getEnclosingScope().resolveLocally(oldSym.getPortName(), EMADynamicPortArraySymbol.KIND);
//            if(!optPort.isPresent()) {
//                if (opt.isPresent()) {
//                    oldSym.setHasPortArray(true);
//                }
//            }
//            if(oldSym.hasPortArray()){
//                int lower = 1;
//                int upper = 1;
//                if(oldSym.hasPortArrayLowerAndUpperBound()){
//                    lower = oldSym.getPortArrayLowerBound();
//                    upper = oldSym.getPortArrayUpperBound();
//                }else if(oldSym.hasPortArraySingleValue()){
//                    lower = oldSym.getPortArraySingleValue();
//                    upper = oldSym.getPortArraySingleValue();
//                }else{
//
//                    if(opt.isPresent()){
//                        upper = opt.get().getDimension();
//                    }
//                }
//
//                if((upper - lower) > 1){
//
//                    List<EventPortExpression> syms = new ArrayList<>();
//
//                    for(int i = 0; i <= (upper - lower); ++i){
//                        EventPortExpressionConnectSymbol sym = new EventPortExpressionConnectSymbol();
//                        sym.setPortName(oldSym.getPortName());
//                        sym.setPortArraySingleValue(lower + i);
//
//                        syms.add(sym);
//                    }
//
//                    return new EventBracketExpressionSymbol(buildAndExpressionFromlist(syms));
//                }else{
//                    EventPortExpressionConnectSymbol sym = new EventPortExpressionConnectSymbol();
//                    sym.setPortName(oldSym.getPortName());
//                    sym.setPortArraySingleValue(lower);
//                    return sym;
//                }
//
//            }else{
//                EventPortExpressionConnectSymbol sym = new EventPortExpressionConnectSymbol();
//                sym.setPortName(oldSym.getPortName());
//                return sym;
//            }
//        }


        return oldSym;
    }

    protected EventExpressionSymbol expandValueExpression(EMADynamicEventHandlerSymbol dehs, EventPortExpressionValueSymbol oldSym){

        if(oldSym.hasComponentName()){
            Collection<EMADynamicComponentInstantiationSymbol> sims = dehs.getEnclosingScope().resolveLocally(EMADynamicComponentInstantiationSymbol.KIND);
            Log.error("Not supported atm.");
        }else{
            Optional<EMADynamicPortArraySymbol> opt = dehs.getEnclosingScope().resolveLocally(oldSym.getPortName(), EMADynamicPortArraySymbol.KIND);
            if(opt.isPresent() && opt.get().getDimension() > 1){
//                opt.get().getDimension();
                oldSym.setHasPortArray(true);
            }
            if(oldSym.hasPortArray()){
                int lower = 1;
                int upper = 1;
                if(oldSym.hasPortArrayLowerAndUpperBound()){
                    lower = oldSym.getPortArrayLowerBound();
                    upper = oldSym.getPortArrayUpperBound();
                }else if(oldSym.hasPortArraySingleValue()){
                    lower = oldSym.getPortArraySingleValue();
                    upper = oldSym.getPortArraySingleValue();
                }else{

                    if(opt.isPresent()){
                        upper = opt.get().getDimension();
                    }
                }

                if((upper - lower) > 1){

                    List<EventPortExpression> syms = new ArrayList<>();

                    for(int i = 0; i <= (upper - lower); ++i){
                        EventPortExpressionValueSymbol sym = new EventPortExpressionValueSymbol();
                        sym.setPortValue(oldSym.getPortValue().getForIndex(i));
                        sym.setPortName(oldSym.getPortName());
                        sym.setPortArraySingleValue(lower + i);

                        syms.add(sym);
                    }

                    return new EventBracketExpressionSymbol(buildAndExpressionFromlist(syms));
                }else{
                    EventPortExpressionValueSymbol sym = new EventPortExpressionValueSymbol();
                    sym.setPortValue(oldSym.getPortValue());
                    sym.setPortName(oldSym.getPortName());
                    sym.setPortArraySingleValue(lower);
                    return sym;
                }

            }else{
                EventPortExpressionValueSymbol sym = new EventPortExpressionValueSymbol();
                sym.setPortValue(oldSym.getPortValue());
                sym.setPortName(oldSym.getPortName());
                return sym;
            }
        }


        return oldSym;
    }

    protected EventLogicalOperationExpressionSymbol buildAndExpressionFromlist(List<EventPortExpression> events){
        if(events.size() == 2){
            return new EventLogicalOperationExpressionSymbol("&&", events.get(0), events.get(1));
        }
        EventPortExpression a = events.get(0);
        events.remove(0);
        return new EventLogicalOperationExpressionSymbol("&&", a, buildAndExpressionFromlist(events));
    }
//</editor-fold>
}
