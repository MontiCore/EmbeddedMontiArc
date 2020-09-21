/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.*;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbolReference;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicEventHandlerSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.helper.SymbolPrinter;

import java.util.*;
import java.util.stream.Collectors;

public class EMADynamicComponentInstanceSymbol extends EMAComponentInstanceSymbol {

    public static final EMADynamicComponentInstanceKIND KIND =  EMADynamicComponentInstanceKIND.INSTANCE;


    protected boolean dynamicInstance = false;


    public EMADynamicComponentInstanceSymbol(String name, EMAComponentSymbolReference type) {
        super(name, type);
        this.setKind(KIND);
    }

    public EMADynamicComponentInstanceSymbol(String name, EMADynamicComponentSymbolReference type){
        super(name, type);
        setKind(KIND);
    }

    public static EMADynamicComponentInstanceBuilder builder() {
        return new EMADynamicComponentInstanceBuilder();
    }


    public boolean isDynamic() {
        return ((EMADynamicComponentSymbolReference)type).getReferencedSymbolAsDynamic().isDynamic();
    }

    @Override
    public String toString() {
//        System.out.println("Now A toString!!!!!!");
        //return super.toString();

        return SymbolPrinter.printDynamicComponentInstance(this);
    }

    public Collection<EMADynamicComponentInstanceSymbol> getDynamicSubComponents() {
        Collection<EMADynamicComponentInstanceSymbol> c = new ArrayList<>();
        this.getSubComponents().forEach(obj -> {
            c.add((EMADynamicComponentInstanceSymbol) obj);
        });
        return c;
    }

    public Collection<EMADynamicEventHandlerInstanceSymbol> getEventHandlers(){
        Collection<EMADynamicEventHandlerInstanceSymbol> c = this.getSpannedScope().<EMADynamicEventHandlerInstanceSymbol>resolveLocally(EMADynamicEventHandlerInstanceSymbol.KIND);

        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }

    @Override
    public Optional<EMAComponentInstanceSymbol> getSubComponent(String name) {
        Optional<EMAComponentInstanceSymbol> comp = this.getSpannedScope().resolveLocally(name, EMAComponentInstanceSymbol.KIND);
        if(comp.isPresent()){
            return comp;
        }

        comp = this.getSpannedScope().resolveLocally(EMAPortInstanceSymbol.getNameWithoutArrayBracketPart(name), EMAComponentInstanceSymbol.KIND);
        if(comp.isPresent()){
            return comp;
        }
        return Optional.empty();
    }

//    TODO: Rework?!
//    public Collection<EMADynamicPortInstanceSymbol> getDynamicPortInstanceSymbolList(){
//        return this.getSpannedScope().resolveLocally(EMADynamicPortInstanceSymbol.KIND);
//    }


    public boolean isDynamicInstance() {
        return dynamicInstance;
    }

    public void setDynamicInstance(boolean dynamicInstance) {
        this.dynamicInstance = dynamicInstance;
    }

    @Override
    public List<EMAConnectorInstanceSymbol> getSubComponentConnectors() {
        Set<EMAConnectorInstanceSymbol> set = new LinkedHashSet<>();

        List<EMAConnectorInstanceSymbol> connectors = new ArrayList<>();
        connectors.addAll(getConnectorInstances());
        for(EMADynamicEventHandlerInstanceSymbol event : this.getEventHandlers()){
            connectors.addAll(event.getConnectorInstances());
        }


        Collection<EMAComponentInstanceSymbol> subComponents = getSubComponents();

        for (EMAConnectorInstanceSymbol connector : connectors) {
            EMAPortInstanceSymbol sourcePort = connector.getSourcePort();
            EMAPortInstanceSymbol targetPort = connector.getTargetPort();
            EMAComponentInstanceSymbol sourceCmp = sourcePort.getComponentInstance();
            EMAComponentInstanceSymbol targetCmp = targetPort.getComponentInstance();
            if (subComponents.contains(sourceCmp) && subComponents.contains(targetCmp)) {
                set.add(connector);
            }

        }

        return new ArrayList<>(set);
    }



    public Collection<EMAConnectorInstanceSymbol> getConnectorInstancesAndEventConnectorInstances() {
        List<EMAConnectorInstanceSymbol> connectors = new ArrayList<>();
        connectors.addAll(getSpannedScope().<EMAConnectorInstanceSymbol>resolveLocally(EMAConnectorInstanceSymbol.KIND));
        for(EMADynamicEventHandlerInstanceSymbol event : this.getEventHandlers()){
            connectors.addAll(event.getConnectorInstances());
        }
        return connectors.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }

    @Override
    public Optional<InstanceInformation> getInstanceInformation() {
        if(this.getName().contains("[")){
            return InstancingRegister.getInstanceInformation(EMAPortSymbol.getNameWithoutArrayBracketPart(this.getName()));
        }
        return InstancingRegister.getInstanceInformation(this.getName());
    }

    @Override
    public String getFullName() {
      return determineFullName();
    }
}
