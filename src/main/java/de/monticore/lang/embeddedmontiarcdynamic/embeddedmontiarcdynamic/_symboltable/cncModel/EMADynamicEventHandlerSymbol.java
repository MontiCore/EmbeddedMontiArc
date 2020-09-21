/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAElementSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicEventHandlerInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventBracketExpressionSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventExpressionSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventLogicalOperationExpressionSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventPortExpression;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.*;

import java.util.*;
import java.util.stream.Collectors;

public class EMADynamicEventHandlerSymbol extends CommonScopeSpanningSymbol implements EMAElementSymbol {

    private static int EventHandlerSymbolID = 0;
    private static int uniqueEventHandlerSymbolID(){
        return ++EventHandlerSymbolID;
    }
    private static String uinqueEventHandlerSymbolName(){
        return "EventHandler["+uniqueEventHandlerSymbolID()+"]";
    }

    public static final EMADynamicEventHandlerSymbolKIND KIND = EMADynamicEventHandlerSymbolKIND.INSTANCE;

    protected EventExpressionSymbol condition;


    //<editor-fold desc="Constructors" >

    public EMADynamicEventHandlerSymbol(){
        super(uinqueEventHandlerSymbolName(), KIND);

    }

    public EMADynamicEventHandlerSymbol(String name){
        super(name, KIND);

    }

    public EMADynamicEventHandlerSymbol(String name, SymbolKind kind) {
        super(name, kind);

    }

    //</editor-fold>

    public EventExpressionSymbol getCondition() {
        return condition;
    }

    public void setCondition(EventExpressionSymbol condition) {
        this.condition = condition;
    }
    public void setConditionWithSymbol(Symbol condition){
        this.condition = (EventExpressionSymbol)condition;
    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder("[");
        sb.append(condition.getTextualRepresentation());
        sb.append("] -> {");
        sb.append("}");

        return sb.toString();
    }

    public Collection<EMADynamicEventHandlerSymbol> getEventHandlers() {

        Collection<EMADynamicEventHandlerSymbol> c = this.getSpannedScope().<EMADynamicEventHandlerSymbol>resolveLocally(EMADynamicEventHandlerSymbol.KIND);

        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());

    }

    public Collection<EMAConnectorSymbol> getConnectors(){
        
        Collection<EMAConnectorSymbol> c = this.getSpannedScope().<EMAConnectorSymbol>resolveLocally(EMAConnectorSymbol.KIND);

        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }

    public Collection<EMAPortSymbol> getPorts(){
        Collection<EMAPortSymbol> ports = this.getSpannedScope().<EMAPortSymbol>resolveLocally(EMAPortSymbol.KIND);

        Collection<EMADynamicEventHandlerSymbol> cEHS = this.getSpannedScope().<EMADynamicEventHandlerSymbol>resolveLocally(EMADynamicEventHandlerSymbol.KIND);

        for (EMADynamicEventHandlerSymbol ehs:cEHS) {
            ports.addAll(ehs.getPorts());
        }

        return ports.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }



    protected void fixCondition(ResolutionDeclarationSymbol resDeclSym){
        this.condition = fixEventExpression(resDeclSym, this.condition);
    }

    protected EventExpressionSymbol fixEventExpression(ResolutionDeclarationSymbol resDeclSym, EventExpressionSymbol ee){
/*        if(ee instanceof EventBracketExpressionSymbol){
            ((EventBracketExpressionSymbol) ee).setInnerExpression(fixEventExpression(resDeclSym, ((EventBracketExpressionSymbol) ee).getInnerExpression()));

        }else if (ee instanceof EventLogicalOperationExpressionSymbol){
            ((EventLogicalOperationExpressionSymbol) ee).setLeftExpression(fixEventExpression(resDeclSym, ((EventLogicalOperationExpressionSymbol) ee).getLeftExpression()));
            ((EventLogicalOperationExpressionSymbol) ee).setRightExpression(fixEventExpression(resDeclSym, ((EventLogicalOperationExpressionSymbol) ee).getRightExpression()));
        }else if(ee instanceof EventPortExpression){
            return fixEventExpressionPortExpression(resDeclSym, (EventPortExpression) ee);
        }*/

        return ee;
    }

    protected EventExpressionSymbol fixEventExpressionPortExpression(ResolutionDeclarationSymbol resDeclSym, EventPortExpression ee){
        return ee;
    }


    public Collection<EMADynamicComponentInstantiationSymbol> getDynamicSubComponents(){
        Collection<EMADynamicComponentInstantiationSymbol> result = this.getSpannedScope().resolveLocally(EMADynamicComponentInstantiationSymbol.KIND);
        return result.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }

    public Collection<EMADynamicPortArraySymbol> getConnectPorts(){
        List<String> names = new ArrayList<>();
        condition.getConnectPortNames(names);

        Scope enclosingScope = this.getEnclosingScope();
        while(!(enclosingScope.getSpanningSymbol().get() instanceof EMADynamicComponentSymbol)){
            enclosingScope = enclosingScope.getSpanningSymbol().get().getEnclosingScope();
        }
        final Scope scope = enclosingScope;
        final Collection<EMADynamicPortArraySymbol> result = new ArrayList<>();
        names.forEach(n -> {
            Optional<EMADynamicPortArraySymbol> pOpt = scope.resolve(n, EMADynamicPortArraySymbol.KIND);
            if(pOpt.isPresent()){
                result.add(pOpt.get());
            }
        });

        return result;
    }
}
