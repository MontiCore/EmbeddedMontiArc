/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.composer;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcArtifactScope;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.emadl.modularcnn.tools.json.JSONBuilder;
import de.monticore.lang.monticar.emadl.modularcnn.tools.json.JSONReader;
import de.monticore.symboltable.references.SymbolReference;

import java.util.ArrayList;
import java.util.Set;

public class NetworkStructureInformation {
    private ArrayList<NetworkStructureInformation> subNetworks = new ArrayList<>();
    private boolean atomic;
    private String networkName;

    private ArrayList<EMAComponentInstanceSymbol> instances = new ArrayList<>();

    private EMAComponentSymbolReference symbolReference = null;
    private String instanceSymbolName;
    private NetworkStructureInformation parentNetwork = null;

    private ArchitectureSymbol composedNetworkArchitectureSymbol = null;
    private String componentName = null;



    public NetworkStructureInformation(ComponentInformation componentInformation) {
        this.networkName = componentInformation.getComponentName();
        this.instanceSymbolName = componentInformation.getComponentInstanceSymbolName();
        ArrayList<ComponentInformation> subComponentsInformation = componentInformation.getSubComponentsInformation();

        if (subComponentsInformation == null || subComponentsInformation.size() == 0) {
            this.atomic = true;
        } else {
            this.atomic = false;
            for (ComponentInformation comp : subComponentsInformation) {
                subNetworks.add(comp.analyzeNetworkStructure());
            }
            rebuildParentRelation();
        }
    }

    public NetworkStructureInformation(String name, String instanceSymbolName, boolean atomic, ArrayList<NetworkStructureInformation> subNets, NetworkStructureInformation parent) {
        this.networkName = name;
        this.instanceSymbolName = instanceSymbolName;
        this.atomic = atomic;
        this.subNetworks = subNets;
        this.parentNetwork = parent;
        rebuildParentRelation();
    }

    public NetworkStructureInformation(String json){
        JSONReader jsonReader = new JSONReader();
        NetworkStructureInformation networkStructureInformation = jsonReader.getNetworkStructureInstance(json);
        this.networkName = networkStructureInformation.getNetworkName();
        this.instanceSymbolName = networkStructureInformation.getInstanceSymbolName();
        this.atomic = networkStructureInformation.isAtomic();
        this.subNetworks = networkStructureInformation.getSubNetworks();
        rebuildParentRelation();
    }

    public void rebuildParentRelation(){
        if (this.subNetworks != null){
            setThisAsParentNetworkInSubnetworks();
            for (NetworkStructureInformation subNet :this.subNetworks){
                subNet.rebuildParentRelation();
            }
        }



    }

    public String getComponentName() {
        return componentName;
    }

    public void setComponentName(String componentName) {
        this.componentName = componentName;
    }

    public ArchitectureSymbol getComposedNetworkArchitectureSymbol(){
        return composedNetworkArchitectureSymbol;
    }
    public void setComposedNetworkArchitectureSymbol(ArchitectureSymbol symbol){
        this.composedNetworkArchitectureSymbol = symbol;
    }

    public EMAComponentSymbolReference getSymbolReference() {
        return symbolReference;
    }

    public void setSymbolReference(EMAComponentSymbolReference symbolReference) {
        this.symbolReference = symbolReference;
    }

    public void addInstance(EMAComponentInstanceSymbol instanceSymbol){
        this.instances.add(instanceSymbol);
    }

    public EMAComponentInstanceSymbol addInstancesAndSymbolReference(Set<EMAComponentInstanceSymbol> componentInstanceSymbols){
        EMAComponentInstanceSymbol hitInstance = null;

        for (EMAComponentInstanceSymbol instanceSymbol: componentInstanceSymbols) {
            String instanceSymbolName = instanceSymbol.getComponentType().getReferencedSymbol().getName();
            if(instanceSymbolName.equals(this.getNetworkName())){
                if (this.symbolReference == null){
                    this.setSymbolReference(instanceSymbol.getComponentType());
                    EmbeddedMontiArcArtifactScope scope = (EmbeddedMontiArcArtifactScope) getSymbolReference().getReferencedSymbol().getEnclosingScope();
                    this.setComponentName(scope.getPackageName() + "." + scope.getName());
                }

                if (!this.instances.contains(instanceSymbol)){
                    this.addInstance(instanceSymbol);
                }

                if (hitInstance == null){
                    hitInstance = instanceSymbol;
                }
            }
            if (this.subNetworks != null && this.subNetworks.size() > 0){
                for (NetworkStructureInformation subNet: this.subNetworks){
                    subNet.addInstancesAndSymbolReference(componentInstanceSymbols);
                }
            }
        }
        return hitInstance;
    }

    public EMAComponentInstanceSymbol refreshInstancesAndSymbolReferences(Set<EMAComponentInstanceSymbol> componentInstanceSymbols){
        EMAComponentInstanceSymbol hitInstance = null;
        this.symbolReference = null;
        this.instances = new ArrayList<>();


        for (EMAComponentInstanceSymbol instanceSymbol: componentInstanceSymbols) {
            String instanceSymbolName = instanceSymbol.getComponentType().getReferencedSymbol().getName();
            if(instanceSymbolName.equals(this.getNetworkName())){
                if (this.symbolReference == null){
                    this.setSymbolReference(instanceSymbol.getComponentType());
                }

                if (!this.instances.contains(instanceSymbol)){
                    this.addInstance(instanceSymbol);
                }

                if (hitInstance == null){
                    hitInstance = instanceSymbol;
                }
            }
            if (this.subNetworks != null && this.subNetworks.size() > 0){
                for (NetworkStructureInformation subNet: this.subNetworks){
                    subNet.refreshInstancesAndSymbolReferences(componentInstanceSymbols);
                }
            }
        }
        return hitInstance;
    }


    public ArrayList<EMAComponentInstanceSymbol> getInstances(){
        return this.instances;
    }

    public boolean isInstancePartOfNetwork(EMAComponentInstanceSymbol instanceSymbol){
        if (this.instances.contains(instanceSymbol)) return true;

        if (this.subNetworks != null && this.subNetworks.size() > 0){
            for (NetworkStructureInformation networkStructureInformation: this.subNetworks){
                if (networkStructureInformation.isInstancePartOfNetwork(instanceSymbol)){
                    return true;
                }
            }
        }
        return false;
    }

    public boolean isInstancePartOfSubNetworks(EMAComponentInstanceSymbol instanceSymbol){
        //if (this.instances.contains(instanceSymbol)) return true;

        if (this.subNetworks != null && this.subNetworks.size() > 0){
            for (NetworkStructureInformation networkStructureInformation: this.subNetworks){
                if (networkStructureInformation.isInstancePartOfNetwork(instanceSymbol)){
                    return true;
                }
            }
        }
        return false;
    }

    private void setThisAsParentNetworkInSubnetworks(){
        if (this.subNetworks == null) return;
        for (NetworkStructureInformation subnet : this.subNetworks){
            subnet.setParentNetwork(this);
        }
    }


    public boolean equals(NetworkStructureInformation net) {
       return equals(net,false);
    }

    public boolean equals(NetworkStructureInformation net, boolean ignoreParent){
        boolean subnetEquality = true;

        if (this.subNetworks == null && net.getSubNetworks() == null) subnetEquality = true;
        else if ((this.subNetworks == null && net.getSubNetworks() != null) || (this.subNetworks != null && net.getSubNetworks() == null) || this.subNetworks.size() != net.getSubNetworks().size())
            subnetEquality = false;
        else {
            for (int i = 0; i < subNetworks.size(); i++) {
                boolean eq = this.subNetworks.get(i).equals(net.getSubNetworks().get(i),true);
                if (!eq) {
                    subnetEquality = false;
                    break;
                }
            }
        }

        boolean parentCheck = true;
        if (!ignoreParent){
            parentCheck = (this.parentNetwork == null && net.getParentNetwork() == null) || (this.parentNetwork != null && net.getParentNetwork() != null && this.parentNetwork.equals(net.getParentNetwork())) ;

        }



        return this.networkName.equals(net.networkName) && this.atomic == net.atomic &&
                this.instanceSymbolName.equals(net.getInstanceSymbolName()) &&
                 parentCheck && subnetEquality;

    }

    public ArrayList<NetworkStructureInformation> getSubNetworks() {
        return subNetworks;
    }

    public boolean isAtomic() {
        return atomic;
    }

    public boolean isRoot(){
        return this.parentNetwork == null;
    }


    public void setParentNetwork(NetworkStructureInformation parentNetwork){
        this.parentNetwork = parentNetwork;
    }

    public NetworkStructureInformation getParentNetwork(){
        return this.parentNetwork;
    }

    public NetworkStructureInformation getRootNetwork(){
        if (this.isRoot()) return this;
        else return this.parentNetwork.getRootNetwork();
    }

    public String getNetworkName() {
        return networkName;
    }

    public String getInstanceSymbolName() {
        return this.instanceSymbolName;
    }

    public String atomicInfo() {
        if (atomic) return "atomic";
        return "composed";
    }

    public String printStructureJSON() {
        if (this.networkName == null || this.networkName.equals("")) {
            return "";
        }

        JSONBuilder jsonObject = new JSONBuilder();

        jsonObject.addContent("name", this.networkName, false);
        jsonObject.addContent("instanceSymbolName",this.instanceSymbolName,false);
        jsonObject.addContent("atomic", this.atomic, false);

        ArrayList<String> arrayContents = new ArrayList<>();
        if (this.getSubNetworks() != null && this.getSubNetworks().size() > 0) {
            for (NetworkStructureInformation networkStructureInformation : this.getSubNetworks()) {
                String subNetContent = networkStructureInformation.printStructureJSON();
                arrayContents.add(subNetContent);
            }

        }

        jsonObject.addContent("subNetworks", arrayContents, true);
        return jsonObject.getJSONObjectAndReset(true);
    }

    public boolean isSubNetOf(ComponentInformation componentInformation) {
        String net = this.printStructureJSON();
        String possibleSupernet = componentInformation.printNetworkStructureJSON();
        return possibleSupernet.contains(net);
    }

    public boolean isSubNetOf(NetworkStructureInformation networkStructureInformation) {
        String net = this.printStructureJSON();
        String possibleSupernet = networkStructureInformation.printStructureJSON();
        return possibleSupernet.contains(net);

    }

    public boolean isSuperNetOf(ComponentInformation componentInformation) {
        String net = this.printStructureJSON();
        String possibleSubnet = componentInformation.printNetworkStructureJSON();
        return net.contains(possibleSubnet);

    }

    public boolean isSuperNetOf(NetworkStructureInformation networkStructureInformation) {
        String net = this.printStructureJSON();
        String possibleSubnet = networkStructureInformation.printStructureJSON();
        return net.contains(possibleSubnet);
    }

    public boolean isComposedNet(String networkName, String instanceName){
        if (this.networkName.equals(networkName) && this.instanceSymbolName.equals(instanceName) && !isAtomic()) return true;
        if (this.isSubnet(networkName,instanceName)){
            NetworkStructureInformation subNet = this.getSubnet(networkName,instanceName);
            return !subNet.isAtomic();
        }
        return false;
    }

    public boolean isSubnet(String networkName, String instanceName){
        if (this.getSubNetworks() == null || this.getSubNetworks().size() == 0) return false;

        for (NetworkStructureInformation subNet: this.getSubNetworks()){
            if (subNet.getNetworkName().equals(networkName) && subNet.getInstanceSymbolName().equals(instanceName)) return true;
            boolean isSub = subNet.isSubnet(networkName,instanceName);
            if (isSub) return true;
        }
        return false;
    }
    public NetworkStructureInformation getSubnet(String networkName, String instanceName){
        if (this.getSubNetworks() == null || this.getSubNetworks().size() == 0) return null;

        for (NetworkStructureInformation subNet: this.getSubNetworks()){
            if (subNet.getNetworkName().equals(networkName) && subNet.getInstanceSymbolName().equals(instanceName)) return subNet;
            NetworkStructureInformation net = subNet.getSubnet(networkName,instanceName);
            if (net != null) return net;
        }

        return null;
    }
}
