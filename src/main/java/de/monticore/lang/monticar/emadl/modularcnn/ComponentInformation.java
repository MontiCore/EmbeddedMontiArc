package de.monticore.lang.monticar.emadl.modularcnn;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentKind;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicConnectorSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class ComponentInformation {
    private ArrayList<ASTInterface> interfaces = new ArrayList<>();
    private ArrayList<ASTConnector> connectors = new ArrayList<>();
    private ArrayList<ASTSubComponent> subComponents = new ArrayList<>();

    private ArrayList<ASTPort> ports = new ArrayList<>();
    private ArrayList<ASTComponent> includedComponents = new ArrayList<>();
    private ArrayList<ComponentInformation> includedComponentsInformation = new ArrayList<>();
    private ArrayList<ConnectorRelation> connectorRelations = new ArrayList<>();

    private String componentName;
    private String componentInstanceName;
    private ComponentKind componentKind;
    private String inputPort;
    private String outputPort;
    private ArrayList<ArchitectureNode> archNodes = null;

    private ASTComponent originalComponentReference = null;

    private boolean violatesNetworkForm = false;

    public ArrayList<ASTInterface> getInterfaces() {
        return interfaces;
    }

    public ArrayList<ASTConnector> getConnectors() {
        return connectors;
    }

    public ArrayList<ASTSubComponent> getSubComponents() {
        return subComponents;
    }

    public ArrayList<ASTPort> getPorts() {
        return ports;
    }

    public ArrayList<ASTComponent> getIncludedComponents() {
        return includedComponents;
    }

    public ArrayList<ComponentInformation> getIncludedComponentsInformation() {
        return includedComponentsInformation;
    }

    public ArrayList<ConnectorRelation> getConnectorRelations() {
        return connectorRelations;
    }

    public String getComponentName() {
        return componentName;
    }

    public String getComponentInstanceName() {
        return componentInstanceName;
    }

    public ComponentKind getComponentKind() {
        return componentKind;
    }

    public String getInputPort() {
        return inputPort;
    }

    public String getOutputPort() {
        return outputPort;
    }

    public ASTComponent getOriginalComponentReference() {
        return originalComponentReference;
    }






    public ComponentInformation(ASTComponent component, ArrayList<ArchitectureNode> currentNodes){


        this.originalComponentReference = component;
        this.componentName = component.getName();
        this.componentInstanceName = "";
        this.archNodes = currentNodes;

        //TODO: add these
        //this.componentKind = component.

        initLists(component);

        printConnectiorRelations();
    }

    public ComponentInformation(ASTComponent component, ArrayList<ArchitectureNode> currentNodes, String instanceName){
        this(component, currentNodes);
        this.componentInstanceName = instanceName;
    }



    //TODO: find actual solution to determine this -> Maybe textfile
    public boolean isASTArchitectureNode(){
        Log.info("ASTArchitecture Check for " + this.getComponentName(),"COMPONENT_INFORMATION");
        for (ArchitectureNode node: archNodes){
            Log.info("Checking if: " + this.componentName + " == " + node.getComponentName(),"COMPONENT_INFORMATION" );
            if (this.componentName.equals(node.getComponentName())) return true;
        }


        return false;
    }

    public boolean isComposedCNN(){
        if (violatesNetworkForm) return false;



        Log.info("Checking included components of " + this.getComponentName(),"COMPONENT_INFORMATION");
        for (ComponentInformation componentInformation: this.includedComponentsInformation){
            Log.info("Checking: " + componentInformation.getComponentName() + " " +componentInformation.getComponentInstanceName(),"COMPONENT_INFORMATION");
            if (!componentInformation.isASTArchitectureNode()) {
                Log.info("failed","COMPONENT_INFORMATION");
                return false;
            }
        }
        Log.info("passed","COMPONENT_INFORMATION");
        return true;
    }

    private void printConnectiorRelations(){
        Log.info("Connector relations of: " + this.componentName + " " + this.componentInstanceName,"COMPONENT_INFORMATION");
        for (ConnectorRelation c : connectorRelations){
            Log.info(c.getSourceValue() + " -> " + c.getTargetValue(),"COMPONENT_INFORMATION");
        }
    }

    private void initLists(ASTComponent component){
        ArrayList<ASTElement> elementList = (ArrayList<ASTElement>) component.getBody().getElementList();
        for (ASTElement e : elementList){
            if (e instanceof ASTInterface){
               this.interfaces.add((ASTInterface) e);
            } else if (e instanceof ASTConnector) {
                this.connectors.add((ASTConnector) e);
            } else if (e instanceof ASTSubComponent) {
                this.subComponents.add((ASTSubComponent) e);
            }
        }

        findPorts(this.interfaces);
        findComponents(this.subComponents);
        findConnectorRelations(this.connectors);
    }

    private void findPorts(ArrayList<ASTInterface> interfaces){
        if (!(interfaces.size() > 0) || interfaces.size() > 1) {
            this.violatesNetworkForm = true;
            return;
        }

        for (ASTInterface i: interfaces){
            ArrayList<ASTPort> interfacePorts = (ArrayList<ASTPort>) i.getPortsList();
            if (interfacePorts.size() > 0) {
                this.ports = interfacePorts;
            }
        }

        if (this.ports.size() != 2) {
            this.violatesNetworkForm = true;
            return;
        }

        for (ASTPort p : this.ports){
            if (p.getNameOpt().isPresent()) {
                if (p.isIncoming() && !p.isOutgoing()) {
                    this.inputPort = p.getNameOpt().get();
                } else if (!p.isIncoming() && p.isOutgoing()){
                    this.outputPort = p.getNameOpt().get();
                }
            }

        }
    }

    private void findComponents(ArrayList<ASTSubComponent> components) {
        if (!(components.size() > 0) ) {
            this.violatesNetworkForm = true;
            return;
        }

        for (ASTSubComponent subComponent: components){
            if (!subComponent.getSymbolOpt().isPresent() || !(subComponent.getSymbolOpt().get() instanceof EMADynamicComponentInstantiationSymbol)){
                continue;
            }

            EMADynamicComponentInstantiationSymbol symbol = (EMADynamicComponentInstantiationSymbol) subComponent.getSymbolOpt().get();
            EMADynamicComponentSymbol refSymbol = (EMADynamicComponentSymbol) symbol.getComponentType().getReferencedSymbol();
            if (refSymbol.getAstNode().isPresent()) {
                this.includedComponents.add((ASTComponent) refSymbol.getAstNode().get());
                this.includedComponentsInformation.add(new ComponentInformation((ASTComponent) refSymbol.getAstNode().get(), this.archNodes, symbol.getName() ));
            }
        }


    }

    private void findConnectorRelations(ArrayList<ASTConnector> connectors){
        for (ASTConnector connector: connectors){
            if (!connector.getSymbolOpt().isPresent()){
                continue;
            }

            String sourceValue = getConnectorSource(connector);
            String targetValue = getConnectorTarget(connector);
            ComponentInformation sourceComponent = matchComponentToPort(sourceValue);
            ComponentInformation targetComponent = matchComponentToPort(targetValue);

            this.connectorRelations.add(new ConnectorRelation(sourceComponent,sourceValue,targetComponent,targetValue));
        }
    }

    //TODO: find correct kind for comparison -> ArchitectureKind or similar to identify CNN declaration
    private void findKind(ASTComponent component){
        if (!component.getSymbolOpt().isPresent()){
            this.violatesNetworkForm = true;
            return;
        }

        EMADynamicComponentSymbol symbol = (EMADynamicComponentSymbol) component.getSymbolOpt().get();
        this.componentKind = (ComponentKind) symbol.getKind();
    }


    private ComponentInformation matchComponentToPort(String portValue){
        if (portValue.equals(this.inputPort) || portValue.equals(this.outputPort)) return this;

        String[] qualifiedNameDecons = portValue.split("\\.");
        String instanceRef = qualifiedNameDecons[0];

        for (ComponentInformation info : this.includedComponentsInformation){
            if (!instanceRef.equals("") && !info.componentInstanceName.equals("") && instanceRef.equals(info.componentInstanceName)){
                return info;
            }
        }

        return null;
    }


    private String getConnectorSource(ASTConnector connector){
        if (!connector.getSymbolOpt().isPresent()) {
            return "Symbol missing for source connector";
        }

        EMADynamicConnectorSymbol symbol = (EMADynamicConnectorSymbol) connector.getSymbolOpt().get();
        return symbol.getSource();
    }

    private String getConnectorTarget(ASTConnector connector){
        if (!connector.getSymbolOpt().isPresent()) {
            return "Symbol missing for target connector";
        }

        EMADynamicConnectorSymbol symbol = (EMADynamicConnectorSymbol) connector.getSymbolOpt().get();
        return symbol.getTarget();
    }


}
