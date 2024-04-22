package de.monticore.lang.monticar.cnnarch.generator.decomposition;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.emadl.modularcnn.compositions.NetworkStructureInformation;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

public class NetworkStructure {

    protected NetworkStructureInformation networkStructureInformation = null;
    protected String networkName = null;
    protected String instanceSymbolName = null;
    protected String componentName = null;
    protected String modelName = null;
    private final ArrayList<LayerInformation> networkLayers = new ArrayList<>();
    private LayerInformation frontSlicePoint = null;
    private LayerInformation backSlicePoint = null;
    private final ArrayList<NetworkStructure> subNetworkStructures = new ArrayList<>();
    private HashMap<String, ArrayList<Integer>> inputPortsDim = new HashMap<>();
    private boolean atomic = false;
    private boolean decompositionAllowed = false;

    public NetworkStructure(NetworkStructureInformation networkStructureInformation) {
        this.networkStructureInformation = networkStructureInformation;
        this.networkName = networkStructureInformation.getNetworkName();
        this.instanceSymbolName = networkStructureInformation.getInstanceSymbolName();
        this.componentName = networkStructureInformation.getComponentName();
        this.atomic = networkStructureInformation.isAtomic();
        this.modelName = null;
        this.inputPortsDim = networkStructureInformation.getInputPortsDim();
    }

    public NetworkStructure(NetworkStructureInformation networkStructureInformation, ArchitectureSymbol architectureSymbol) {
        this(networkStructureInformation);
        processArchitectureSymbol(architectureSymbol);
    }

    public boolean isAtomic() {
        return this.atomic;
    }

    public boolean isDecompositionAllowed() {
        return this.decompositionAllowed;
    }

    public void setDecompositionAllowed(boolean allowed) {
        this.decompositionAllowed = allowed;
    }

    public String getNetworkName() {
        return networkName;
    }

    public String getInstanceSymbolName() {
        return instanceSymbolName;
    }

    public String getComponentName() {
        return componentName;
    }

    public NetworkStructureInformation getNetworkStructureInformation() {
        return networkStructureInformation;
    }

    public ArrayList<LayerInformation> getNetworkLayers() {
        return this.networkLayers;
    }

    public HashMap<String, ArrayList<Integer>> getInputPortsDim() {
        return inputPortsDim;
    }

    public void addNetworkLayer(LayerInformation networkLayer) {
        this.networkLayers.add(networkLayer);
    }

    public void setDecompositionControl(String[] allowedNetworks) {
        if (allowedNetworks == null || allowedNetworks.length == 0) return;

        for (int i = 0; i < allowedNetworks.length; i++) {
            if (allowedNetworks[i].equals(this.networkName) && !this.isAtomic()) {
                this.setDecompositionAllowed(true);
                break;
            } else {
                this.setDecompositionAllowed(false);
            }
        }

        for (NetworkStructure subnet : this.subNetworkStructures) {
            subnet.setDecompositionControl(allowedNetworks);
        }
    }

    public void addFrontSlicePoint(LayerInformation networkLayer) {
        this.networkLayers.add(0, networkLayer);
        this.frontSlicePoint = networkLayer;
    }

    public void addBackSlicePoint(LayerInformation networkLayer) {
        this.networkLayers.add(networkLayer);
        this.backSlicePoint = networkLayer;
    }

    public boolean hasSubnet(NetworkStructure network) {
        if (!this.subNetworkStructures.isEmpty()) {
            for (NetworkStructure subnet : this.subNetworkStructures) {
                if (subnet.atomic == network.atomic
                        && subnet.getNetworkName().equals(network.getNetworkName())
                        && subnet.getComponentName().equals(network.getComponentName())) {
                    return true;
                } else if (subnet.hasSubnet(network)) {
                    return true;
                }
            }
        }
        return false;
    }

    public void reassignSubnet(NetworkStructure network) {
        if (!this.subNetworkStructures.isEmpty()) {
            for (int i = 0; i < this.subNetworkStructures.size(); i++) {
                NetworkStructure subnet = this.subNetworkStructures.get(i);
                if (subnet.atomic == network.atomic
                        && subnet.getNetworkName().equals(network.getNetworkName())
                        && subnet.getComponentName().equals(network.getComponentName())) {
                    this.subNetworkStructures.set(i, network);
                } else if (subnet.hasSubnet(network)) {
                    subnet.reassignSubnet(network);
                }
            }
        }
    }

    public ArrayList<NetworkStructure> getSubNetworkStructures() {
        return subNetworkStructures;
    }

    public ArrayList<NetworkStructure> getNetsToDecompose() {
        ArrayList<NetworkStructure> netsToDecompose = new ArrayList<>();
        for (NetworkStructure subnet : this.subNetworkStructures) {
            if (subnet.isAtomic()) {
                netsToDecompose.add(subnet);
            } else if (!subnet.isAtomic() && !subnet.decompositionAllowed) {
                netsToDecompose.add(subnet);
            } else {
                netsToDecompose.addAll(subnet.getNetsToDecompose());
            }
        }
        return netsToDecompose;
    }

    public void addPrecedingSubNetwork(NetworkStructure atomicNetworkStructure) {
        this.subNetworkStructures.add(0, atomicNetworkStructure);
    }

    public void addPrecedingSubNetworks(ArrayList<NetworkStructure> atomicNetworkStructuresList) {
        this.subNetworkStructures.addAll(0, atomicNetworkStructuresList);
    }

    public void addSucceedingSubNetwork(NetworkStructure atomicNetworkStructure) {
        this.subNetworkStructures.add(atomicNetworkStructure);
    }

    public void addSucceedingSubNetworks(ArrayList<NetworkStructure> atomicNetworkStructuresList) {
        this.subNetworkStructures.addAll(atomicNetworkStructuresList);
    }

    private void processArchitectureSymbol(ArchitectureSymbol architectureSymbol) {
        List<NetworkInstructionSymbol> networkInstructions = architectureSymbol.getNetworkInstructions();

        for (NetworkInstructionSymbol instructionSymbol : networkInstructions) {
            List<ArchitectureElementSymbol> elementSymbols = instructionSymbol.getBody().getElements();
            for (int i = 0; i < elementSymbols.size(); i++) {
                ArchitectureElementSymbol elementSymbol = elementSymbols.get(i);

                if (elementSymbol instanceof LayerSymbol) {
                    processLayerSymbol((LayerSymbol) elementSymbol, i, elementSymbols.size());
                } else if (elementSymbol instanceof ParallelCompositeElementSymbol) {
                    processParallelCompositeElement((ParallelCompositeElementSymbol) elementSymbol, i, elementSymbols.size());
                } else if (elementSymbol instanceof VariableSymbol) {
                    if (!elementSymbol.getInputElement().isPresent() && elementSymbol.getOutputElement().isPresent()) {
                        addLayer(new LayerInformation(elementSymbol.getName(), LayerType.INPUT, elementSymbol), i, elementSymbols.size());
                    } else if (elementSymbol.getInputElement().isPresent() && !elementSymbol.getOutputElement().isPresent()) {
                        addLayer(new LayerInformation(elementSymbol.getName(), LayerType.OUTPUT, elementSymbol), i, elementSymbols.size());
                    }
                } else {
                    Log.error("Unexpected Layer Element Type " + elementSymbol.getClass().getSimpleName());
                }
            }
        }
    }

    private void processLayerSymbol(LayerSymbol layerSymbol, int i, int totalElements) {
        if (layerSymbol.getDeclaration().getBody() == null) {
            addLayer(new LayerInformation(layerSymbol.getName(), LayerType.DEFAULT, layerSymbol), i, totalElements);
            return;
        }
        for (ArchitectureElementSymbol subElement : layerSymbol.getDeclaration().getBody().getElements()) {
            if (subElement instanceof ParallelCompositeElementSymbol) {
                processParallelCompositeElement((ParallelCompositeElementSymbol) subElement, i, totalElements);
            } else if (subElement instanceof LayerSymbol) {
                LayerInformation layerInfo = new LayerInformation(subElement.getName(), LayerType.DEFAULT, subElement);
                addLayer(layerInfo, i, totalElements);
            } else {
                Log.error("Unexpected Layer Element Type " + subElement.getClass().getSimpleName());
            }
        }
    }

    private void processParallelCompositeElement(ParallelCompositeElementSymbol parallelElement, int index, int totalElements) {
        List<List<String>> parallelNames = extractNamesFromParallelElement(parallelElement);
        LayerInformation parallelLayerInfo = new LayerInformation(parallelNames, LayerType.PARALLEL_DEFAULT, parallelElement);
        addLayer(parallelLayerInfo, index, totalElements);
    }

    private List<List<String>> extractNamesFromParallelElement(ParallelCompositeElementSymbol parallelElement) {
        List<List<String>> parallelStreams = new ArrayList<>();
        for (ArchitectureElementSymbol element : parallelElement.getElements()) {
            if (element instanceof SerialCompositeElementSymbol) {
                List<String> names = new ArrayList<>();
                for (ArchitectureElementSymbol subElement : ((SerialCompositeElementSymbol) element).getElements()) {
                    names.add(subElement.getName());
                }
                parallelStreams.add(names);
            } else {
                parallelStreams.add(Collections.singletonList(element.getName()));
            }
        }
        return parallelStreams;
    }

    private void addLayer(LayerInformation layerInformation, int index, int totalElements) {
        if (layerInformation.isInputLayer() && index == 0) {
            this.frontSlicePoint = layerInformation;
        } else if (layerInformation.isOutputLayer() && index == totalElements - 1) {
            this.backSlicePoint = layerInformation;
        }
        if (!this.networkLayers.isEmpty()) {
            LayerInformation lastLayer = this.networkLayers.get(this.networkLayers.size() - 1);
            lastLayer.setSucceedingLayer(layerInformation);
            layerInformation.setPreceedingLayer(lastLayer);
        }
        this.networkLayers.add(layerInformation);
        Log.debug("Processed " + layerInformation, "NetworkStructure");
    }
}