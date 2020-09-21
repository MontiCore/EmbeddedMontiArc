/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValueSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValueSymbolHelper;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.types.references.ActualTypeArgument;

import java.util.Map;

public class EventExpressionSymbolHelper {

    //<editor-fold desc="Single Instance">
    protected static EventExpressionSymbolHelper INSTANCE = null;
    public static EventExpressionSymbolHelper getINSTANCE(){
        if(INSTANCE == null){
            EventExpressionSymbolHelper.init();
        }
        return INSTANCE;
    }
    protected void setINSTANCE(EventExpressionSymbolHelper builder){
        INSTANCE = builder;
    }
    public static EventExpressionSymbolHelper init(){
        EventExpressionSymbolHelper b = new EventExpressionSymbolHelper();
        b.setINSTANCE(b);
        return b;
    }
    //</editor-fold>

    protected void expandWithArgsAndParams(EventExpressionSymbol ees, Map<MCTypeSymbol, ActualTypeArgument> args, Map<EMAVariable, PortValueSymbol> params){
        if(ees == null)
        {
            return;
        }
        if(ees instanceof EventPortExpression){
            EventPortExpression epe = (EventPortExpression) ees;
            args.forEach((k,v) -> {
                if(!epe.hasComponentName() && epe.portName.equals(k.getName())){
                    epe.setPortName(v.getType().getName());
                }
            });
            if(ees instanceof EventPortExpressionValueSymbol){
                PortValueSymbol pvs = PortValueSymbolHelper.getINSTANCE().expand(((EventPortExpressionValueSymbol) ees).getPortValue(), params);
                ((EventPortExpressionValueSymbol) ees).setPortValue(pvs);
            }
            return;
        }

        //EventBooleanExpressionSymbol not needed

        if(ees instanceof EventBracketExpressionSymbol){
            expandWithArgsAndParams(((EventBracketExpressionSymbol) ees).getInnerExpression(), args, params);
        }
        if(ees instanceof EventLogicalOperationExpressionSymbol){
            expandWithArgsAndParams(((EventLogicalOperationExpressionSymbol) ees).getLeftExpression(), args, params);
            expandWithArgsAndParams(((EventLogicalOperationExpressionSymbol) ees).getRightExpression(),args, params);
        }
    }
}
