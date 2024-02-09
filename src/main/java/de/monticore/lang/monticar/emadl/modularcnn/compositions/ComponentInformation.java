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
package de.monticore.lang.monticar.emadl.modularcnn.compositions;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentKind;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicConnectorSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstantiationSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.emadl.modularcnn.tools.ScopeFinder;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class ComponentInformation {
    final private ArrayList<ASTInterface> interfaces = new ArrayList<>();
    final private ArrayList<ASTConnector> connectors = new ArrayList<>();
    final private ArrayList<ASTSubComponent> astSubComponents = new ArrayList<>();
    final private ArrayList<ASTComponent> includedComponents = new ArrayList<>();
    final private ArrayList<ComponentInformation> subComponentsInformation = new ArrayList<>();
    final private ArrayList<DataflowRelation> dataflowRelations = new ArrayList<>();
    final private ASTComponent originalComponentReference;
    final private String componentName;
    final private ArrayList<ArchitectureNode> archNodes;
    private ArrayList<ASTPort> ports = new ArrayList<>();
    private String componentInstanceSymbolName;
    private ComponentKind componentKind;
    private ArrayList<String> inputPorts = new ArrayList<>();
    private HashMap<String, ArrayList<Integer>> inputPortsDim = new HashMap<>();
    private ArrayList<String> outputPorts = new ArrayList<>();
    private boolean violatesNetworkForm = false;
    private boolean isCNNNode = false;

    private ArrayList<String> connectorFlow = new ArrayList<>();
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

    public ArrayList<DataflowRelation> getConnectorRelations() {
        return dataflowRelations;
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

    public ArrayList<String> getInputPorts() {
        return inputPorts;
    }

    public HashMap<String, ArrayList<Integer>> getInputPortsDim() {return inputPortsDim;}

    public ArrayList<String> getOutputPorts() {
        return outputPorts;
    }

    public ASTComponent getOriginalComponentReference() {
        return originalComponentReference;
    }


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
        this.connectorFlow = analyzeConnectorFlow();

    }

    private boolean isASTArchitectureNode() {
        for (ArchitectureNode node : archNodes) {
            if (this.componentName.equals(node.getComponentName()) && !node.isComposedNode()) return true;
        }
        return false;
    }

    public boolean isCNN() {
        return this.isCNNNode;
    }

    public boolean isComposedCNN() {
        if (violatesNetworkForm) {
            return false;
        }

        for (ComponentInformation componentInformation : this.subComponentsInformation) {
            if (!componentInformation.isCNN()) {
                return false;
            }
        }

        return true;
    }

    public NetworkStructureInformation analyzeNetworkStructure(){
        return new NetworkStructureInformation(this);
    }

    public String printNetworkStructureJSON(){
        return analyzeNetworkStructure().printStructureJSON();
    }

    public ArrayList<String> getConnectorFlow(){


        return this.connectorFlow;
    }

    private ArrayList<String> analyzeConnectorFlow(){
        if (this.dataflowRelations == null){
            return new ArrayList<>();
        }
        ArrayList<String> flow = new ArrayList<>();

        String previousSource = "";
        String previousTarget = "";

        for ( DataflowRelation relation : this.dataflowRelations) {
            if (relation.getSource() == null || relation.getTarget() == null) continue;
            String source = relation.getSource().getComponentName()  + "|" + relation.getSource().getComponentInstanceSymbolName();
            String target = relation.getTarget().getComponentName()  + "|" + relation.getTarget().getComponentInstanceSymbolName();

            if (flow.size() == 0) {
                flow.add(source);
            } else if (!flow.get(flow.size() - 1).equals(source)) {
                if (!previousSource.equals(source)){
                    flow.add(source);
                }
            }

            if (!flow.get(flow.size()-1).equals(target)) {
                if (!previousTarget.equals(target)){
                    flow.add(target);
                }

            }

            previousSource = source;
            previousTarget = target;
        }
        return flow;
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

        if (instanceName != null && !instanceName.equals("")) this.componentInstanceSymbolName = instanceName.toLowerCase();
    }

    private void printConnectiorRelations() {
        Log.info("Connector relations of: " + this.componentName + " " + this.componentInstanceSymbolName, "COMPONENT_INFORMATION");
        for (DataflowRelation c : dataflowRelations) {
            Log.info(c.getSourceValue() + " -> " + c.getTargetValue(), "COMPONENT_INFORMATION");
        }
    }

    private void findPorts(ArrayList<ASTInterface> interfaces){
        if (this.interfaces.isEmpty()){
            this.violatesNetworkForm = true;
        }

        for (ASTInterface i : interfaces) {
            ArrayList<ASTPort> interfacePorts = (ArrayList<ASTPort>) i.getPortsList();
            if (!interfacePorts.isEmpty()) {
                this.ports = interfacePorts;
            }
        }

        if (this.ports.isEmpty()){
            this.violatesNetworkForm = true;
        }

        for (ASTPort p : this.ports) {
            if (!p.getNameOpt().isPresent()) {
                continue;
            }
            String portName = p.getNameOpt().get();
            if (p.isIncoming() && !p.isOutgoing()) {
                this.inputPorts.add(portName);
                this.inputPortsDim.put(portName, extractPortDimensions(p));
            } else if (!p.isIncoming() && p.isOutgoing()) {
                this.outputPorts.add(portName);
            }
        }
    }

    private ArrayList<Integer> extractPortDimensions(ASTPort port) {
        ArrayList<Integer> dimensions = new ArrayList<>();

        if (!(port.getType() instanceof ASTCommonMatrixType)) {
            return dimensions;
        }
        ASTCommonMatrixType matrixType = (ASTCommonMatrixType) port.getType();
        List<ASTExpression> expressions = matrixType.getDimension().getDimensionList();
        for (ASTExpression expression : expressions) {
            if (expression instanceof ASTNumberExpression) {
                ASTNumberWithUnit numberWithUnit = ((ASTNumberExpression) expression).getNumberWithUnit();
                numberWithUnit.getNumber().ifPresent(number -> dimensions.add(number.intValue()));
            }
        }
        return dimensions;
    }

    private void findComponents(ArrayList<ASTSubComponent> components) {
        if (!(components.size() > 1)) {
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
                ASTComponent astComponent = (ASTComponent) refSymbol.getAstNode().get();
                ComponentInformation subCompInfo = new ComponentInformation(astComponent, this.archNodes);

                if (subCompInfo.isCNNNode ){
                    this.includedComponents.add(astComponent);
                    this.subComponentsInformation.add(subCompInfo);
                } else {
                    this.violatesNetworkForm = true;
                }
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

            this.dataflowRelations.add(new DataflowRelation(sourceComponent, sourceValue, targetComponent, targetValue));
        }
    }

    private ComponentInformation matchComponentToPort(String portValue) {
        if (this.inputPorts.contains(portValue) || this.outputPorts.contains(portValue)) return this;

        String[] qualifiedNameDecons = portValue.split("\\.", 2);
        String instanceRef = qualifiedNameDecons[0];

        for (ComponentInformation info : this.subComponentsInformation) {
            if (!instanceRef.equals("") && !info.componentInstanceSymbolName.equals("") && (instanceRef.equals(info.componentInstanceSymbolName) || instanceRef.equals(info.componentInstanceSymbolName.toLowerCase()) )) {
                return info.matchComponentToPort(qualifiedNameDecons[1]);
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
