/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicConnectorSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicPortArraySymbol;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.ScopeSpanningSymbol;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class EMADynamicConnectorInstanceSymbol extends EMAConnectorInstanceSymbol {

    public static final EMADynamicConnectorInstanceKIND KIND = EMADynamicConnectorInstanceKIND.INSTANCE;

    protected boolean dynamicSourceNewComponent;
    protected boolean dynamicSourceNewPort;
    protected boolean dynamicTargetNewComponent;
    protected boolean dynamicTargetNewPort;

    public EMADynamicConnectorInstanceSymbol(String name) {
        super(name);
        this.setKind(KIND);
    }

    //<editor-fold desc="Dynamic Part">
    public boolean isDynamicSourceNewComponent() {
        return dynamicSourceNewComponent;
    }

    public void setDynamicSourceNewComponent(boolean dynamicSourceNewComponent) {
        this.dynamicSourceNewComponent = dynamicSourceNewComponent;
    }

    public boolean isDynamicSourceNewPort() {
        return dynamicSourceNewPort;
    }

    public void setDynamicSourceNewPort(boolean dynamicSourceNewPort) {
        this.dynamicSourceNewPort = dynamicSourceNewPort;
    }

    public boolean isDynamicTargetNewComponent() {
        return dynamicTargetNewComponent;
    }

    public void setDynamicTargetNewComponent(boolean dynamicTargetNewComponent) {
        this.dynamicTargetNewComponent = dynamicTargetNewComponent;
    }

    public boolean isDynamicTargetNewPort() {
        return dynamicTargetNewPort;
    }

    public void setDynamicTargetNewPort(boolean dynamicTargetNewPort) {
        this.dynamicTargetNewPort = dynamicTargetNewPort;
    }

    public boolean hasDynamicNew(){
        return isDynamicTargetNewPort() || isDynamicSourceNewPort() || isDynamicTargetNewComponent() || isDynamicSourceNewComponent();
    }
    //</editor-fold>

    //<editor-fold desc="Maybe not working. Wrong!">
    @Override
    protected EMAPortInstanceSymbol getPort(String name) {

        if(name.contains("?")){
            name = name.replace("?", "1");
        }

        if (this.getEnclosingScope() == null) {
            Log.warn("ConnectorInstance does not belong to a componentInstance, cannot resolve port");
            return null;
        } else if (!this.getEnclosingScope().getSpanningSymbol().isPresent()) {
            Log.warn("ConnectorInstance is not embedded in an expanded component instance symbol, cannot resolve port");
            return null;
        } else {

            Scope enclosingScope = this.getEnclosingScope();
            if(enclosingScope.getSpanningSymbol().get() instanceof EMADynamicEventHandlerInstanceSymbol){
                enclosingScope = enclosingScope.getSpanningSymbol().get().getEnclosingScope();
            }

            String fullSource = Joiners.DOT.join(((ScopeSpanningSymbol)enclosingScope.getSpanningSymbol().get()).getFullName(), name, new Object[0]);
            Optional<EMAPortInstanceSymbol> port = enclosingScope.resolve(fullSource, EMAPortInstanceSymbol.KIND);
            if (port.isPresent()) {
                return (EMAPortInstanceSymbol)port.get();
            } else {
                Iterator<String> parts = Splitters.DOT.split(name).iterator();
                Log.debug("" + name, "NAME:");
                if (!parts.hasNext()) {
                    Log.warn("name of connector's source/target is empty, cannot resolve port");
                    return null;
                } else {
                    String instance = (String)parts.next();
                    Log.debug("" + instance, "instance");
                    if (!parts.hasNext()) {
                        Log.warn("name of connector's source/target does has two parts: instance.port, cannot resolve port");
                        return null;
                    } else {
                        String instancePort = (String)parts.next();
                        Log.debug("" + instancePort, "instancePort");
                        Optional<EMAComponentInstanceSymbol> inst = ((ScopeSpanningSymbol)enclosingScope.getSpanningSymbol().get()).getSpannedScope().resolve(instance, EMAComponentInstanceSymbol.KIND);
                        if (!inst.isPresent()) {
                            Log.warn(String.format("Could not find instance %s in connector scope, cannot resolve port", instance));
                            return null;
                        } else {
                            port = ((EMAComponentInstanceSymbol)inst.get()).getSpannedScope().resolve(instancePort, EMAPortInstanceSymbol.KIND);
                            if (port.isPresent()) {
                                return (EMAPortInstanceSymbol)port.get();
                            } else {
                                Log.debug("No case match for" + name, "cannot resolve port");
                                return null;
                            }
                        }
                    }
                }
            }
        }
    }

    //</editor-fold>

    //<editor-fold desc="Instantiate">

    public void instantiate(EMAConnectorSymbol connector, String packageName){
        //this should be in a parent class
        this.setSource(connector.getSource());
        this.setTarget(connector.getTarget());
        this.setIsConstantConnector(connector.isConstant());
        if(connector.isConstant()) {
            this.setConstantEMAPortSymbol(connector.getSourcePort());
        }
        this.setPackageName(packageName);
        this.setFullName(packageName + "." + this.getName());

        //set ast node for ordering of connercot instances!
        this.setAstNode(connector.getAstNode().orElse(null));

        if(connector instanceof EMADynamicConnectorSymbol){
            this.setDynamicSourceNewComponent(((EMADynamicConnectorSymbol) connector).isDynamicSourceNewComponent());
            this.setDynamicSourceNewPort(((EMADynamicConnectorSymbol) connector).isDynamicSourceNewPort());
            this.setDynamicTargetNewComponent(((EMADynamicConnectorSymbol) connector).isDynamicTargetNewComponent());
            this.setDynamicTargetNewPort(((EMADynamicConnectorSymbol) connector).isDynamicTargetNewPort());
        }
    }

    public static EMADynamicConnectorInstanceSymbol newAndInstantiate(EMAConnectorSymbol connector, String packageName){
       /* EMADynamicPortInstanceSymbol p = new EMADynamicPortInstanceSymbol(port.getName());
        p.instantiate(port, packageName);
        return p;*/
        EMADynamicConnectorInstanceSymbol connectorInstance = new EMADynamicConnectorInstanceSymbol(connector.getTarget());
        connectorInstance.instantiate(connector, packageName);
        return connectorInstance;

    }

    //</editor-fold>

    public Optional<String> getSourceComponentName(){
        if(this.getSource().contains(".")){
            return Optional.of(this.getSource().substring(0, this.getSource().indexOf(".")));
        }
        return Optional.empty();
    }

    public String getSourcePortName(){
        if(this.getSource().contains(".")){
            return this.getSource().substring(this.getSource().indexOf(".")+1);
        }
        return this.getSource();
    }

    public Map<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> getSources(){
        Map<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> map = new HashMap<>();
        if(this.isDynamicSourceNewComponent()){
            //instance[?].port
            String compName = EMAPortSymbol.getNameWithoutArrayBracketPart(this.getSourceComponentName().get());
            String portName = this.getSourcePort().getName();

            Scope enclosingScope = this.getEnclosingScope();
            if(enclosingScope.getSpanningSymbol().get() instanceof EMADynamicEventHandlerInstanceSymbol){
                enclosingScope = enclosingScope.getSpanningSymbol().get().getEnclosingScope();
            }

            int id = 1;
            Optional<EMAComponentInstanceSymbol> sym = enclosingScope.<EMAComponentInstanceSymbol>resolve(compName+"["+id+"]", EMAComponentInstanceSymbol.KIND);
            while(sym.isPresent()){
                if(sym.get() instanceof EMADynamicComponentInstanceSymbol && ((EMADynamicComponentInstanceSymbol) sym.get()).isDynamicInstance()) {
                    List<EMAPortInstanceSymbol> l = new ArrayList<>();
                    l.add(sym.get().getPortInstance(portName).get());
                    map.put(sym.get(), l);
                }
                ++id;
                sym = enclosingScope.<EMAComponentInstanceSymbol>resolve(compName+"["+id+"]", EMAComponentInstanceSymbol.KIND);
            }
        }else if(this.isDynamicSourceNewPort()){
            EMAPortInstanceSymbol port = this.getSourcePort();
            String portName = port.getNameWithoutArrayBracketPart();

            EMAComponentInstanceSymbol sym = port.getComponentInstance();

            int id = 1;
            List<EMAPortInstanceSymbol> l = new ArrayList<>();
            Optional<EMAPortInstanceSymbol> portInstance = sym.getPortInstance(portName+"["+id+"]");
            while(portInstance.isPresent()){
                if(portInstance.get() instanceof EMADynamicPortInstanceSymbol){
                    if(((EMADynamicPortInstanceSymbol) portInstance.get()).isDynamic()){
                        l.add(portInstance.get());
                    }
                }
                ++id;
                portInstance = sym.getPortInstance(portName+"["+id+"]");
            }

            map.put(sym, l);
        }else{
            EMAPortInstanceSymbol p = this.getSourcePort();
            List<EMAPortInstanceSymbol> l = new ArrayList<>();
            l.add(p);
            map.put(p.getComponentInstance(), l);
        }

        return map;
    }

    public Collection<EMAPortInstanceSymbol> getSourcesPorts(){
        Collection<EMAPortInstanceSymbol> result = new ArrayList<>();
        Map<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> s = getSources();
        for(Map.Entry<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> e : s.entrySet()){
            result.addAll(e.getValue());
        }
        return result;
    }

    public Map<String, EMAPortInstanceSymbol> getSourcePortsMap(){
        Map<String, EMAPortInstanceSymbol> ports = new HashMap<>();
        if(this.isDynamicSourceNewComponent()){
            String compName = EMAPortSymbol.getNameWithoutArrayBracketPart(this.getSourceComponentName().get());
            String portName = this.getSourcePort().getName();

            Scope enclosingScope = this.getEnclosingScope();
            if(enclosingScope.getSpanningSymbol().get() instanceof EMADynamicEventHandlerInstanceSymbol){
                enclosingScope = enclosingScope.getSpanningSymbol().get().getEnclosingScope();
            }
            int id = 1;
            Optional<EMAComponentInstanceSymbol> sym = enclosingScope.<EMAComponentInstanceSymbol>resolve(compName+"["+id+"]", EMAComponentInstanceSymbol.KIND);
            while(sym.isPresent()){
                if(sym.get() instanceof EMADynamicComponentInstanceSymbol && ((EMADynamicComponentInstanceSymbol) sym.get()).isDynamicInstance()) {
                    Optional<EMAPortInstanceSymbol> pI = sym.get().getPortInstance(portName);
                    if(pI.isPresent()) {
                        ports.put(compName + "[" + id + "]." + portName, pI.get());
                    }
                }
                ++id;
                sym = enclosingScope.<EMAComponentInstanceSymbol>resolve(compName+"["+id+"]", EMAComponentInstanceSymbol.KIND);
            }
        }else if(this.isDynamicSourceNewPort()){
            String fname = EMAPortSymbol.getNameWithoutArrayBracketPart(this.getSource());
            EMAPortInstanceSymbol port = this.getSourcePort();
            String portName = port.getNameWithoutArrayBracketPart();

            EMAComponentInstanceSymbol sym = port.getComponentInstance();

            int id = 1;
            Optional<EMAPortInstanceSymbol> portInstance = sym.getPortInstance(portName+"["+id+"]");
            while(portInstance.isPresent()){
                if(portInstance.get() instanceof EMADynamicPortInstanceSymbol){
                    if(((EMADynamicPortInstanceSymbol) portInstance.get()).isDynamic()){
                        ports.put(fname+"["+id+"]", portInstance.get());
                    }
                }
                ++id;
                portInstance = sym.getPortInstance(portName+"["+id+"]");
            }
        }else {
            ports.put(this.getSource(), this.getSourcePort());
        }

        return ports;
    }

    public List<String> getAllDynamicSourceNames(){
        List<String> names = new ArrayList<>();
        if(this.isDynamicSourceNewComponent()){
            String compName = EMAPortSymbol.getNameWithoutArrayBracketPart(this.getSourceComponentName().get());
            String portName = this.getSourcePort().getName();

            Scope enclosingScope = this.getEnclosingScope();
            if(enclosingScope.getSpanningSymbol().get() instanceof EMADynamicEventHandlerInstanceSymbol){
                enclosingScope = enclosingScope.getSpanningSymbol().get().getEnclosingScope();
            }
            int id = 1;
            Optional<EMAComponentInstanceSymbol> sym = enclosingScope.<EMAComponentInstanceSymbol>resolve(compName+"["+id+"]", EMAComponentInstanceSymbol.KIND);
            while(sym.isPresent()){
                if(sym.get() instanceof EMADynamicComponentInstanceSymbol && ((EMADynamicComponentInstanceSymbol) sym.get()).isDynamicInstance()) {
                    names.add(compName+"["+id+"]."+portName);
                }
                ++id;
                sym = enclosingScope.<EMAComponentInstanceSymbol>resolve(compName+"["+id+"]", EMAComponentInstanceSymbol.KIND);
            }
        }else if(this.isDynamicSourceNewPort()){
            String fname = EMAPortSymbol.getNameWithoutArrayBracketPart(this.getSource());
            EMAPortInstanceSymbol port = this.getSourcePort();
            String portName = port.getNameWithoutArrayBracketPart();

            EMAComponentInstanceSymbol sym = port.getComponentInstance();

            int id = 1;
            Optional<EMAPortInstanceSymbol> portInstance = sym.getPortInstance(portName+"["+id+"]");
            while(portInstance.isPresent()){
                if(portInstance.get() instanceof EMADynamicPortInstanceSymbol){
                    if(((EMADynamicPortInstanceSymbol) portInstance.get()).isDynamic()){
                        names.add(fname+"["+id+"]");
                    }
                }
                ++id;
                portInstance = sym.getPortInstance(portName+"["+id+"]");
            }
        }else {
            names.add(this.getSource());
        }

        return names;
    }

    public Optional<String> getTargetComponentName(){
        if(this.getTarget().contains(".")){
            return Optional.of(this.getTarget().substring(0, this.getTarget().indexOf(".")));
        }
        return Optional.empty();
    }

    public Collection<EMADynamicComponentInstanceSymbol> getTargetDynamicNewComponents(){
        Optional<String> target = getTargetComponentName();
        if(!target.isPresent()){
            return new ArrayList<>();
        }
        String name = EMAPortSymbol.getNameWithoutArrayBracketPart(target.get());
        Scope enclosingScope = this.getEnclosingScope();
        if(enclosingScope.getSpanningSymbol().get() instanceof EMADynamicEventHandlerInstanceSymbol){
            enclosingScope = enclosingScope.getSpanningSymbol().get().getEnclosingScope();
        }

        List<EMADynamicComponentInstanceSymbol> l = new ArrayList<>();
        int id = 1;
        Optional<EMADynamicComponentInstanceSymbol> inst = enclosingScope.<EMADynamicComponentInstanceSymbol>resolve(name+"["+id+"]", EMADynamicComponentInstanceSymbol.KIND);
        while(inst.isPresent()){
            if(inst.get().isDynamicInstance()){
                l.add(inst.get());
            }
            ++id;
            inst = enclosingScope.<EMADynamicComponentInstanceSymbol>resolve(name+"["+id+"]", EMADynamicComponentInstanceSymbol.KIND);
        }

        return l;
    }

    public Map<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> getTargets(){
        Map<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> map = new HashMap<>();
        if(this.isDynamicTargetNewComponent()){
            //instance[?].port
            String compName = EMAPortSymbol.getNameWithoutArrayBracketPart(this.getTargetComponentName().get());
            String portName = this.getTargetPort().getName();

            Scope enclosingScope = this.getEnclosingScope();
            if(enclosingScope.getSpanningSymbol().get() instanceof EMADynamicEventHandlerInstanceSymbol){
                enclosingScope = enclosingScope.getSpanningSymbol().get().getEnclosingScope();
            }

            int id = 1;
            Optional<EMAComponentInstanceSymbol> sym = enclosingScope.<EMAComponentInstanceSymbol>resolve(compName+"["+id+"]", EMAComponentInstanceSymbol.KIND);
            while(sym.isPresent()){

                if(sym.get() instanceof EMADynamicComponentInstanceSymbol && ((EMADynamicComponentInstanceSymbol) sym.get()).isDynamicInstance()) {
                    List<EMAPortInstanceSymbol> l = new ArrayList<>();
                    l.add(sym.get().getPortInstance(portName).get());
                    map.put(sym.get(), l);
                }
                ++id;
                sym = enclosingScope.<EMAComponentInstanceSymbol>resolve(compName+"["+id+"]", EMAComponentInstanceSymbol.KIND);
            }
        }else if(this.isDynamicTargetNewPort()){
            EMAPortInstanceSymbol port = this.getTargetPort();
            String portName = port.getNameWithoutArrayBracketPart();

            EMAComponentInstanceSymbol sym = port.getComponentInstance();

            int id = 1;
            List<EMAPortInstanceSymbol> l = new ArrayList<>();
            Optional<EMAPortInstanceSymbol> portInstance = sym.getPortInstance(portName+"["+id+"]");
            while(portInstance.isPresent()){
                if(portInstance.get() instanceof EMADynamicPortInstanceSymbol){
                    if(((EMADynamicPortInstanceSymbol) portInstance.get()).isDynamic()){
                        l.add(portInstance.get());
                    }
                }
                ++id;
                portInstance = sym.getPortInstance(portName+"["+id+"]");
            }

            map.put(sym, l);
        }else{
            EMAPortInstanceSymbol p = this.getTargetPort();
            List<EMAPortInstanceSymbol> l = new ArrayList<>();
            l.add(p);
            map.put(p.getComponentInstance(), l);
        }

        return map;
    }

    public Collection<EMAPortInstanceSymbol> getTargetsPorts(){
        Collection<EMAPortInstanceSymbol> result = new ArrayList<>();
        Map<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> s = getTargets();
        for(Map.Entry<EMAComponentInstanceSymbol, List<EMAPortInstanceSymbol>> e : s.entrySet()){
            result.addAll(e.getValue());
        }
        return result;
    }

    public List<String> getAllDynamicTargetNames(){
        List<String> names = new ArrayList<>();
        if(this.isDynamicTargetNewComponent()){
            String compName = EMAPortSymbol.getNameWithoutArrayBracketPart(this.getTargetComponentName().get());
            String portName = this.getTargetPort().getName();

            Scope enclosingScope = this.getEnclosingScope();
            if(enclosingScope.getSpanningSymbol().get() instanceof EMADynamicEventHandlerInstanceSymbol){
                enclosingScope = enclosingScope.getSpanningSymbol().get().getEnclosingScope();
            }
            int id = 1;
            Optional<EMAComponentInstanceSymbol> sym = enclosingScope.<EMAComponentInstanceSymbol>resolve(compName+"["+id+"]", EMAComponentInstanceSymbol.KIND);
            while(sym.isPresent()){
                if(sym.get() instanceof EMADynamicComponentInstanceSymbol && ((EMADynamicComponentInstanceSymbol) sym.get()).isDynamicInstance()) {
                    names.add(compName+"["+id+"]."+portName);
                }
                ++id;
                sym = enclosingScope.<EMAComponentInstanceSymbol>resolve(compName+"["+id+"]", EMAComponentInstanceSymbol.KIND);
            }
        }else if(this.isDynamicTargetNewPort()){
            String fname = EMAPortSymbol.getNameWithoutArrayBracketPart(this.getTarget());
            EMAPortInstanceSymbol port = this.getTargetPort();
            String portName = port.getNameWithoutArrayBracketPart();

            EMAComponentInstanceSymbol sym = port.getComponentInstance();

            int id = 1;
            Optional<EMAPortInstanceSymbol> portInstance = sym.getPortInstance(portName+"["+id+"]");
            while(portInstance.isPresent()){
                if(portInstance.get() instanceof EMADynamicPortInstanceSymbol){
                    if(((EMADynamicPortInstanceSymbol) portInstance.get()).isDynamic()){
                        names.add(fname+"["+id+"]");
                    }
                }
                ++id;
                portInstance = sym.getPortInstance(portName+"["+id+"]");
            }
        }else {
            names.add(this.getTarget());
        }

        return names;
    }


    public String getTargetPortName(){
        if(this.getTarget().contains(".")){
            return this.getTarget().substring(this.getTarget().indexOf(".")+1);
        }
        return this.getTarget();
    }

    public Collection<EMAConnectorInstanceSymbol> getAllPossibleConnectors(){

        Collection<EMAConnectorInstanceSymbol> connects = new ArrayList<>();


        List<String> sources  = getAllDynamicSourceNames();
        List<String> targets = getAllDynamicTargetNames();
        for(String s : sources){
            for(String t : targets){
                EMADynamicConnectorInstanceSymbol connectorInstance = new EMADynamicConnectorInstanceSymbol(t);

                connectorInstance.setSource(s);
                connectorInstance.setTarget(t);
                connectorInstance.setIsConstantConnector(this.isConstant());
                if(this.isConstant()) {
                    connectorInstance.setConstantEMAPortSymbol(this.getSourcePort());
                }
                connectorInstance.setPackageName(this.getPackageName());
                connectorInstance.setFullName(this.getPackageName() + "." + connectorInstance.getName());

                //ast node!
                connectorInstance.setAstNode(this.getAstNode().orElse(null));

                connects.add(connectorInstance);
            }
        }

        return connects;
    }

    public Collection<EMAConnectorInstanceSymbol> getAllPossibleConnectsWithSourcePort(EMAPortInstanceSymbol sourcePort){
        Collection<EMAConnectorInstanceSymbol> connects = new ArrayList<>();

        String s = sourcePort.getName();
        if(this.getSourceComponentName().isPresent()){
            s = sourcePort.getComponentInstance().getName()+"."+s;
        }

        for(String t : getAllDynamicTargetNames()){
            EMADynamicConnectorInstanceSymbol connectorInstance = new EMADynamicConnectorInstanceSymbol(t);

            connectorInstance.setSource(s);
            connectorInstance.setTarget(t);
            connectorInstance.setIsConstantConnector(this.isConstant());
            if(this.isConstant()) {
                connectorInstance.setConstantEMAPortSymbol(this.getSourcePort());
            }
            connectorInstance.setPackageName(this.getPackageName());
            connectorInstance.setFullName(this.getPackageName() + "." + connectorInstance.getName());

            //ast node
            connectorInstance.setAstNode(this.getAstNode().orElse(null));
            connects.add(connectorInstance);
        }

        return connects;
    }
}
