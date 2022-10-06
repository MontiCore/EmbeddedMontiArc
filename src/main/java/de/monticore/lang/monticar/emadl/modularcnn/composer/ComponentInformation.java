/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.composer;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentKind;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicConnectorSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.monticore.lang.monticar.emadl.modularcnn.tools.ScopeFinder;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Set;

public class ComponentInformation {
    final private ArrayList<ASTInterface> interfaces = new ArrayList<>();
    final private ArrayList<ASTConnector> connectors = new ArrayList<>();
    final private ArrayList<ASTSubComponent> astSubComponents = new ArrayList<>();
    final private ArrayList<ASTComponent> includedComponents = new ArrayList<>();
    final private ArrayList<ComponentInformation> subComponentsInformation = new ArrayList<>();
    final private ArrayList<ConnectorRelation> connectorRelations = new ArrayList<>();
    final private ASTComponent originalComponentReference;
    final private String componentName;
    final private ArrayList<ArchitectureNode> archNodes;
    private ArrayList<ASTPort> ports = new ArrayList<>();
    private String componentInstanceSymbolName;
    private ComponentKind componentKind;
    private String inputPort;
    private String outputPort;
    private boolean violatesNetworkForm = false;
    private boolean isCNNNode = false;

    public ArrayList<ASTInterface> getInterfaces() {
        return interfaces;
    }

    public ArrayList<ASTConnector> getConnectors() {
        return connectors;
    }

    public ArrayList<ASTSubComponent> getAstSubComponents() {
        return astSubComponents;
    }

    public ArrayList<ASTPort> getPorts() {
        return ports;
    }

    public ArrayList<ASTComponent> getIncludedComponents() {
        return includedComponents;
    }

    public ArrayList<ComponentInformation> getSubComponentsInformation() {
        return subComponentsInformation;
    }

    public ArrayList<ConnectorRelation> getConnectorRelations() {
        return connectorRelations;
    }

    public String getComponentName() {
        return componentName;
    }

    public String getComponentInstanceSymbolName() {
        return componentInstanceSymbolName;
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

    //TODO: CHECK
    public ComponentInformation(ASTComponent component, ArrayList<ArchitectureNode> currentNodes) {
        this.originalComponentReference = component;
        this.componentName = component.getName();
        this.componentInstanceSymbolName = "";
        this.archNodes = currentNodes;

        initLists(component);

        if (this.isComposedCNN()) {
            ArrayList<ASTArchitecture> subNodes = new ArrayList<>();
            for (ComponentInformation componentInformation : this.subComponentsInformation) {
                for (ArchitectureNode architectureNode : this.archNodes) {
                    if (componentInformation.getComponentName().equals(architectureNode.getComponentName()) && !subNodes.contains(architectureNode.getOriginalNode()) && architectureNode.getOriginalNodes() != null) {
                        subNodes.addAll(architectureNode.getOriginalNodes());
                    }
                }
            }
            ArchitectureNode architectureNode = new ArchitectureNode(subNodes, this.componentName);
            this.archNodes.add(architectureNode);
        }

        this.isCNNNode = this.isASTArchitectureNode() || this.isComposedCNN();
        findInstanceSymbolNameIfEmpty();
    }

    /*
    public ComponentInformation(ASTComponent component, ArrayList<ArchitectureNode> currentNodes, String instanceName) {
        this(component, currentNodes);
        //this.componentInstanceName = instanceName;
        findInstanceNameIfEmpty();
    }
    */



    private boolean isASTArchitectureNode() {
        for (ArchitectureNode node : archNodes) {
            if (this.componentName.equals(node.getComponentName()) && !node.isComposedNode()) return true;
        }
        return false;
    }

    public boolean isCNN() {
        return this.isCNNNode;
    }

    //TODO: CHECK
    public boolean isComposedCNN() {
        Log.info("Checking included components of " + this.getComponentName(), "COMPONENT_INFORMATION_CCNN_CHECK");
        if (violatesNetworkForm) {
            Log.info("FORM_VIOLATION_BY: " + this.getComponentName(), "COMPONENT_INFORMATION_CCNN_CHECK");
            return false;
        }

        for (ComponentInformation componentInformation : this.subComponentsInformation) {
            Log.info("Checking: " + componentInformation.getComponentName() + " " + componentInformation.getComponentInstanceSymbolName(), "COMPONENT_INFORMATION_CCNN_CHECK");
            if (!componentInformation.isCNN()) {
                Log.info("IS_COMPOSED_CHECK_FAIL", "COMPONENT_INFORMATION_CCNN_CHECK");
                return false;
            }
        }

        Log.info("IS_COMPOSED_CHECK_PASS", "COMPONENT_INFORMATION_CCNN_CHECK");
        return true;
    }

    public NetworkStructureInformation analyzeNetworkStructure(){
        return new NetworkStructureInformation(this);
    }

    public String printNetworkStructureJSON(){
        return analyzeNetworkStructure().printStructureJSON();
    }

    private void initLists(ASTComponent component) {
        ArrayList<ASTElement> elementList = (ArrayList<ASTElement>) component.getBody().getElementList();
        for (ASTElement e : elementList) {
            if (e instanceof ASTInterface) {
                this.interfaces.add((ASTInterface) e);
            } else if (e instanceof ASTConnector) {
                this.connectors.add((ASTConnector) e);
            } else if (e instanceof ASTSubComponent) {
                this.astSubComponents.add((ASTSubComponent) e);
            }
        }

        findPorts(this.interfaces);
        findComponents(this.astSubComponents);
        findConnectorRelations(this.connectors);
    }

    private void findInstanceSymbolNameIfEmpty(){
        if (!this.componentInstanceSymbolName.equals("")) return;

        ScopeFinder scopeFinder = new ScopeFinder();
        Set<String> keySet = scopeFinder.getNextArtifactScopeUp(this.getOriginalComponentReference().getEnclosingScope()).getLocalSymbols().keySet();
        String instanceName = null;
        for (Iterator<String> keyIterator = keySet.iterator(); keyIterator.hasNext();){
            instanceName = keyIterator.next();
        }

        if (instanceName != null && !instanceName.equals("")) this.componentInstanceSymbolName = instanceName;
    }

    private void printConnectiorRelations() {
        Log.info("Connector relations of: " + this.componentName + " " + this.componentInstanceSymbolName, "COMPONENT_INFORMATION");
        for (ConnectorRelation c : connectorRelations) {
            Log.info(c.getSourceValue() + " -> " + c.getTargetValue(), "COMPONENT_INFORMATION");
        }
    }

    private void findPorts(ArrayList<ASTInterface> interfaces){
        if (interfaces.size() != 1) {
            Log.info("FORM_VIOLATION: " + this.getComponentName(),"COMPONENT_INFORMATION_FIND_PORTS_1");
            this.violatesNetworkForm = true;
            return;
        }

        for (ASTInterface i : interfaces) {
            ArrayList<ASTPort> interfacePorts = (ArrayList<ASTPort>) i.getPortsList();
            if (interfacePorts.size() > 0) {
                this.ports = interfacePorts;
            }
        }

        if (this.ports.size() != 2) {
            Log.info("FORM_VIOLATION: " + this.getComponentName(),"COMPONENT_INFORMATION_FIND_PORTS_2");
            this.violatesNetworkForm = true;
            return;
        }

        for (ASTPort p : this.ports) {
            if (p.getNameOpt().isPresent()) {
                if (p.isIncoming() && !p.isOutgoing()) {
                    this.inputPort = p.getNameOpt().get();
                } else if (!p.isIncoming() && p.isOutgoing()) {
                    this.outputPort = p.getNameOpt().get();
                }
            }
        }
    }

    //TODO: CHECK
    private void findComponents(ArrayList<ASTSubComponent> components) {
        if (!(components.size() > 0)) {
            Log.info("FORM_VIOLATION: " + this.getComponentName(),"COMPONENT_INFORMATION_FIND_COMPONENTS");
            this.violatesNetworkForm = true;
            return;
        }

        for (ASTSubComponent subComponent : components) {
            if (!subComponent.getSymbolOpt().isPresent() || !(subComponent.getSymbolOpt().get() instanceof EMADynamicComponentInstantiationSymbol)) {
                continue;
            }

            EMADynamicComponentInstantiationSymbol symbol = (EMADynamicComponentInstantiationSymbol) subComponent.getSymbolOpt().get();
            EMADynamicComponentSymbol refSymbol = (EMADynamicComponentSymbol) symbol.getComponentType().getReferencedSymbol();
            if (refSymbol.getAstNode().isPresent()) {
                this.includedComponents.add((ASTComponent) refSymbol.getAstNode().get());
                this.subComponentsInformation.add(new ComponentInformation((ASTComponent) refSymbol.getAstNode().get(), this.archNodes));
            }
        }
    }

    private void findConnectorRelations(ArrayList<ASTConnector> connectors) {
        for (ASTConnector connector : connectors) {
            if (!connector.getSymbolOpt().isPresent()) {
                continue;
            }

            String sourceValue = getConnectorSource(connector);
            String targetValue = getConnectorTarget(connector);
            ComponentInformation sourceComponent = matchComponentToPort(sourceValue);
            ComponentInformation targetComponent = matchComponentToPort(targetValue);

            this.connectorRelations.add(new ConnectorRelation(sourceComponent, sourceValue, targetComponent, targetValue));
        }
    }

    private ComponentInformation matchComponentToPort(String portValue) {
        if (portValue.equals(this.inputPort) || portValue.equals(this.outputPort)) return this;

        String[] qualifiedNameDecons = portValue.split("\\.");
        String instanceRef = qualifiedNameDecons[0];

        for (ComponentInformation info : this.subComponentsInformation) {
            if (!instanceRef.equals("") && !info.componentInstanceSymbolName.equals("") && instanceRef.equals(info.componentInstanceSymbolName)) {
                return info;
            }
        }
        return null;
    }

    private String getConnectorSource(ASTConnector connector) {
        if (!connector.getSymbolOpt().isPresent()) {
            return "Symbol missing for source connector: " + connector.toString();
        }

        EMADynamicConnectorSymbol symbol = (EMADynamicConnectorSymbol) connector.getSymbolOpt().get();
        return symbol.getSource();
    }

    private String getConnectorTarget(ASTConnector connector) {
        if (!connector.getSymbolOpt().isPresent()) {
            return "Symbol missing for target connector: " + connector.toString();
        }

        EMADynamicConnectorSymbol symbol = (EMADynamicConnectorSymbol) connector.getSymbolOpt().get();
        return symbol.getTarget();
    }
}
