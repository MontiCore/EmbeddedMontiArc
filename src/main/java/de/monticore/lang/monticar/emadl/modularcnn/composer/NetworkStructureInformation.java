/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.composer;

import de.monticore.lang.monticar.emadl.modularcnn.tools.json.JSONBuilder;
import de.monticore.lang.monticar.emadl.modularcnn.tools.json.JSONReader;

import java.util.ArrayList;

public class NetworkStructureInformation {
    private ArrayList<NetworkStructureInformation> subNetworks = new ArrayList<>();
    private boolean atomic;
    private String networkName;

    private String instanceName;
    private NetworkStructureInformation parentNetwork = null;

    public NetworkStructureInformation(ComponentInformation componentInformation) {
        this.networkName = componentInformation.getComponentName();
        this.instanceName = componentInformation.getComponentInstanceName();
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

    public NetworkStructureInformation(String name, String instanceName, boolean atomic, ArrayList<NetworkStructureInformation> subNets, NetworkStructureInformation parent) {
        this.networkName = name;
        this.instanceName = instanceName;
        this.atomic = atomic;
        this.subNetworks = subNets;
        this.parentNetwork = parent;
        rebuildParentRelation();
    }

    public NetworkStructureInformation(String json){
        JSONReader jsonReader = new JSONReader();
        NetworkStructureInformation networkStructureInformation = jsonReader.getNetworkStructureInstance(json);
        this.networkName = networkStructureInformation.getNetworkName();
        this.instanceName = networkStructureInformation.getInstanceName();
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
                this.instanceName.equals(net.getInstanceName()) &&
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

    public String getInstanceName() {
        return this.instanceName;
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
        jsonObject.addContent("instanceName",this.instanceName,false);
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
        if (this.networkName.equals(networkName) && this.instanceName.equals(instanceName) && !isAtomic()) return true;
        if (this.isSubnet(networkName,instanceName)){
            NetworkStructureInformation subNet = this.getSubnet(networkName,instanceName);
            return !subNet.isAtomic();
        }
        return false;
    }

    public boolean isSubnet(String networkName, String instanceName){
        if (this.getSubNetworks() == null || this.getSubNetworks().size() == 0) return false;

        for (NetworkStructureInformation subNet: this.getSubNetworks()){
            if (subNet.getNetworkName().equals(networkName) && subNet.getInstanceName().equals(instanceName)) return true;
            boolean isSub = subNet.isSubnet(networkName,instanceName);
            if (isSub) return true;
        }
        return false;
    }
    public NetworkStructureInformation getSubnet(String networkName, String instanceName){
        if (this.getSubNetworks() == null || this.getSubNetworks().size() == 0) return null;

        for (NetworkStructureInformation subNet: this.getSubNetworks()){
            if (subNet.getNetworkName().equals(networkName) && subNet.getInstanceName().equals(instanceName)) return subNet;
            NetworkStructureInformation net = subNet.getSubnet(networkName,instanceName);
            if (net != null) return net;
        }

        return null;
    }
}
