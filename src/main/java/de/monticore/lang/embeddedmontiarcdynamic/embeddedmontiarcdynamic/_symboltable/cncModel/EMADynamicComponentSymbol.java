/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentKind;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationKind;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.helper.SymbolPrinter;
import de.monticore.symboltable.SymbolKind;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class EMADynamicComponentSymbol extends EMAComponentSymbol {

    protected boolean isDynamic;


    public EMADynamicComponentSymbol(String name) {
        super(name);
        isDynamic = false;
    }

    public EMADynamicComponentSymbol(String name, SymbolKind kind) {
        super(name, kind);
        isDynamic = false;
    }

    public Collection<EMADynamicEventHandlerSymbol> getEventHandlers() {

        Collection<EMADynamicEventHandlerSymbol> c = this.getReferencedComponent().orElse(this)
                .getSpannedScope().<EMADynamicEventHandlerSymbol>resolveLocally(EMADynamicEventHandlerSymbol.KIND);

        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());

    }

    public Collection<EMADynamicComponentSymbol> getInnerDynamicComponents(){
        Collection<EMADynamicComponentSymbol> c = this.getReferencedComponent().orElse(this)
                .getSpannedScope().<EMADynamicComponentSymbol>resolveLocally(EMADynamicComponentSymbol.KIND);

        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }

    public Collection<EMADynamicComponentInstantiationSymbol> getDynamicSubComponents(){
        Collection<EMAComponentInstantiationSymbol> c = this.getSubComponents();
        List<EMADynamicComponentInstantiationSymbol> col = new ArrayList<>();

        c.forEach(obj -> {
            if(obj instanceof EMADynamicComponentInstantiationSymbol){
                col.add((EMADynamicComponentInstantiationSymbol)obj);
            }
        });

        return col;
    }

    public Collection<EMADynamicConnectorSymbol> getDynamicConnectors(){
        Collection<EMAConnectorSymbol> c = this.getConnectors();
        Collection<EMADynamicConnectorSymbol> dc = new ArrayList<>();
        c.forEach(obj -> {
            dc.add((EMADynamicConnectorSymbol)obj);
        });
        return dc;
    }

    @Override
    public Collection<EMAPortSymbol> getPortsList() {
        Collection<EMAPortSymbol> c = new ArrayList<>(super.getPortsList());
        c.addAll(this.getDynamicPortArraysList());

        Collection<EMADynamicEventHandlerSymbol> cEHS = getEventHandlers();
        for(EMADynamicEventHandlerSymbol ehs : cEHS){
            c.addAll(ehs.getPorts());
        }
        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }

    @Override
    public Collection<EMAPortArraySymbol> getPortArrays() {
        Collection<EMAPortArraySymbol> c = new ArrayList<>(super.getPortArrays());

        c.addAll(getDynamicPortArraysList());

        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }

    public Collection<EMADynamicPortArraySymbol> getDynamicPortArraysList() {
        Collection<EMADynamicPortArraySymbol> c = this.getReferencedComponent().orElse(this)
                .getSpannedScope().<EMADynamicPortArraySymbol>resolveLocally(EMADynamicPortArraySymbol.KIND);
        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }

    public boolean isDynamic() {
        return isDynamic;
    }

    public void setDynamic(boolean dynamic) {
        isDynamic = dynamic;
    }

    @Override
    public String toString() {
        return SymbolPrinter.printDynamicComponent(this);
    }

    @Override
    public String getFullName() {
        return determineFullName();
    }
}
