/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceKind;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAElementInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicConnectorSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicEventHandlerSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.kind.EMADynamicEventHandlerInstanceKIND;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventExpressionSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventPortExpressionConnectSymbol;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.SymbolKind;

import java.util.Collection;
import java.util.stream.Collectors;

public class EMADynamicEventHandlerInstanceSymbol extends EMADynamicEventHandlerSymbol implements EMAElementInstanceSymbol {

    public static final EMADynamicEventHandlerInstanceKIND KIND = EMADynamicEventHandlerInstanceKIND.INSTANCE;


    public EMADynamicEventHandlerInstanceSymbol(String name) {
        super(name, KIND);
    }

    protected EMADynamicEventHandlerInstanceSymbol(String name, SymbolKind kind) {
        super(name, kind);
    }

    @Override
    public Collection<EMAConnectorSymbol> getConnectors(){
        Scope s = this.getSpannedScope();
        Collection<EMAConnectorSymbol> c = s.<EMAConnectorSymbol>resolveLocally(EMAConnectorInstanceSymbol.KIND);

        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }

    public Collection<EMAConnectorInstanceSymbol> getConnectorInstances(){
        Scope s = this.getSpannedScope();
        Collection<EMAConnectorInstanceSymbol> c = s.<EMAConnectorInstanceSymbol>resolveLocally(EMAConnectorInstanceSymbol.KIND);

        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }




    public Collection<EMADynamicConnectorInstanceSymbol> getConnectorsDynamic(){
        Scope s = this.getSpannedScope();
        Collection<EMADynamicConnectorInstanceSymbol> c = s.<EMADynamicConnectorInstanceSymbol>resolveLocally(EMADynamicConnectorInstanceSymbol.KIND);

        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }

    public boolean isDynamicPortConnectionEvent(){
        return this.getCondition().hasConnectSymbol();
    }

    public boolean isDynamicPortFreeEvent(){
        return this.getCondition().hasFreeSymbol();
    }
}
