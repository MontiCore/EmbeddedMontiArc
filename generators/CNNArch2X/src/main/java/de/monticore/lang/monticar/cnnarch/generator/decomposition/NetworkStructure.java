package de.monticore.lang.monticar.cnnarch.generator.decomposition;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.emadl.modularcnn.compositions.NetworkStructureInformation;

import java.util.ArrayList;
import java.util.List;

public class NetworkStructure {

    protected NetworkStructureInformation networkStructureInformation = null;
    protected String networkName = null;
    protected String instanceSymbolName = null;
    protected String componentName = null;
    protected String modelName = null;
    private ArrayList<LayerInformation> networkLayers = new ArrayList<>();
    private LayerInformation frontSlicePoint = null;
    private LayerInformation backSlicePoint = null;

    private ArrayList<NetworkStructure> subNetworkStructures = new ArrayList<>();
    private boolean atomic = false;

    private boolean decompositionAllowed = false;

    public NetworkStructure(NetworkStructureInformation networkStructureInformation){
        this.networkStructureInformation = networkStructureInformation;
        this.networkName = networkStructureInformation.getNetworkName();
        this.instanceSymbolName = networkStructureInformation.getInstanceSymbolName();
        this.componentName = networkStructureInformation.getComponentName();
        this.atomic = networkStructureInformation.isAtomic();
        this.modelName = null;
    }

    public NetworkStructure(NetworkStructureInformation networkStructureInformation, ArchitectureSymbol architectureSymbol){
        this(networkStructureInformation);
        processArchitectureSymbol(architectureSymbol);
    }

    public boolean isAtomic(){
        return this.atomic;
    }

    public boolean isDecompositionAllowed(){
        return this.decompositionAllowed;
    }

    public void setDecompositionAllowed(boolean allowed){
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

    public ArrayList<LayerInformation> getNetworkLayers(){
        return this.networkLayers;
    }

    public void addNetworkLayer(LayerInformation networkLayer){
        this.networkLayers.add(networkLayer);
    }

    public void setDecompositionControl(String[] allowedNetworks){
        if (allowedNetworks == null || allowedNetworks.length == 0) return;

        for (int i=0; i<allowedNetworks.length; i++){
            if (allowedNetworks[i].equals(this.networkName) && !this.isAtomic()) {
                this.setDecompositionAllowed(true);
                break;
            }
            else {
                this.setDecompositionAllowed(false);
            }
        }

        for (NetworkStructure subnet: this.subNetworkStructures){
            subnet.setDecompositionControl(allowedNetworks);
        }
    }

    public void addFrontSlicePoint(LayerInformation networkLayer){
        this.networkLayers.add(0,networkLayer);
        this.frontSlicePoint = networkLayer;
    }

    public void addBackSlicePoint(LayerInformation networkLayer){
        this.networkLayers.add(networkLayer);
        this.backSlicePoint = networkLayer;
    }

    public boolean hasSubnet(NetworkStructure network){
        if (this.subNetworkStructures.size() > 0) {
            for (int i=0; i<this.subNetworkStructures.size(); i++){
                NetworkStructure subnet = this.subNetworkStructures.get(i);
                if (subnet.atomic == network.atomic
                        && subnet.getNetworkName().equals(network.getNetworkName())
                        && subnet.getComponentName().equals(network.getComponentName())){
                    return true;
                } else if (subnet.hasSubnet(network)){
                    return true;
                }
            }
        }
        return false;
    }

    public void reassignSubnet(NetworkStructure network){
        if (this.subNetworkStructures.size() > 0) {
            for (int i=0; i<this.subNetworkStructures.size(); i++){
                NetworkStructure subnet = this.subNetworkStructures.get(i);
                if (subnet.atomic == network.atomic
                        && subnet.getNetworkName().equals(network.getNetworkName())
                        && subnet.getComponentName().equals(network.getComponentName())){
                    this.subNetworkStructures.set(i,network);
                } else if (subnet.hasSubnet(network)){
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
        for (NetworkStructure subnet: this.subNetworkStructures){
            if (subnet.isAtomic()){
                netsToDecompose.add(subnet);
            } else if (!subnet.isAtomic() && !subnet.decompositionAllowed) {
                netsToDecompose.add(subnet);
            } else {
                netsToDecompose.addAll(subnet.getNetsToDecompose());
            }
        }
        return netsToDecompose;
    }

    public void addPrecedingSubNetwork(NetworkStructure atomicNetworkStructure){
        this.subNetworkStructures.add(0,atomicNetworkStructure);
    }

    public void addPrecedingSubNetworks(ArrayList<NetworkStructure> atomicNetworkStructuresList){
        this.subNetworkStructures.addAll(0,atomicNetworkStructuresList);
    }

    public void addSucceedingSubNetwork(NetworkStructure atomicNetworkStructure){
        this.subNetworkStructures.add(atomicNetworkStructure);
    }

    public void addSucceedingSubNetworks(ArrayList<NetworkStructure> atomicNetworkStructuresList){
        this.subNetworkStructures.addAll(atomicNetworkStructuresList);
    }

    private void processArchitectureSymbol(ArchitectureSymbol architectureSymbol){
        List<NetworkInstructionSymbol> networkInstructions = architectureSymbol.getNetworkInstructions();

        for (NetworkInstructionSymbol symbol : networkInstructions){

            List<ArchitectureElementSymbol> elementSymbols = symbol.getBody().getElements();
            for (int i=0; i<elementSymbols.size();i++){
                ArchitectureElementSymbol elementSymbol = elementSymbols.get(i);
                LayerType layerType = null;
                boolean parallelSymbol = false;
                ArrayList<String> parallelNames = null;

                if (elementSymbol instanceof VariableSymbol){
                    if (i==0) layerType = LayerType.INPUT;
                    else layerType = LayerType.OUTPUT;
                } else if (elementSymbol instanceof ParallelCompositeElementSymbol){
                    if (i==0) layerType = LayerType.PARALLEL_INPUT;
                    else if (i == elementSymbols.size()-1) layerType = LayerType.PARALLEL_OUTPUT;
                    else layerType = LayerType.PARALLEL_DEFAULT;

                    parallelSymbol = true;
                    parallelNames = new ArrayList<>();

                    ParallelCompositeElementSymbol parallelCompositeElementSymbol = (ParallelCompositeElementSymbol) elementSymbol;

                    for (ArchitectureElementSymbol architectureElementSymbol : parallelCompositeElementSymbol.getElements()){
                        SerialCompositeElementSymbol serialCompositeElementSymbol = (SerialCompositeElementSymbol) architectureElementSymbol;
                        ArrayList<ArchitectureElementSymbol> varSymbols = (ArrayList<ArchitectureElementSymbol>) serialCompositeElementSymbol.getElements();
                        VariableSymbol varSym = (VariableSymbol) varSymbols.get(0);
                        parallelNames.add(varSym.getName());
                    }

                } else if (elementSymbol instanceof LayerSymbol) {
                    LayerSymbol layerSymbol = (LayerSymbol) elementSymbol;
                    if (layerSymbol.getDeclaration().getBody() != null){
                        for (ArchitectureElementSymbol funcSymbol: layerSymbol.getDeclaration().getBody().getElements()) {
                            LayerSymbol funcLayerSymbol = (LayerSymbol) funcSymbol;
                            LayerInformation funcLayer = new LayerInformation(funcLayerSymbol.getName(), LayerType.DEFAULT, funcSymbol);
                            this.addNetworkLayer(funcLayer);
                        }
                        continue;
                        //layerType = LayerType.FUNCTION;
                    } else {
                        layerType = LayerType.DEFAULT;
                    }
                }

                LayerInformation layerInformation = null;
                if (parallelSymbol){
                    layerInformation = new LayerInformation(parallelNames, layerType, elementSymbol);
                } else {
                    layerInformation = new LayerInformation(elementSymbol.getName(), layerType, elementSymbol);
                }

                if (layerInformation.isInputLayer() || layerInformation.isOutputLayer()){
                    if (i==0) this.frontSlicePoint = layerInformation;
                    else this.backSlicePoint = layerInformation;
                }
                if (this.networkLayers.size() > 0){
                    this.networkLayers.get(this.networkLayers.size()-1).setSucceedingLayer(layerInformation);
                    layerInformation.setPreceedingLayer(this.networkLayers.get(this.networkLayers.size()-1));
                }

                this.addNetworkLayer(layerInformation);
            }
        }
    }
}
