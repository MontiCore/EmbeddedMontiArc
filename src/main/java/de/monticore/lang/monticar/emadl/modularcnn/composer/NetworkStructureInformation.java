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
        }
    }

    public NetworkStructureInformation(String name, String instanceName, boolean atomic, ArrayList<NetworkStructureInformation> subNets) {
        this.networkName = name;
        this.instanceName = instanceName;
        this.atomic = atomic;
        this.subNetworks = subNets;
    }

    public NetworkStructureInformation(String json){
        JSONReader jsonReader = new JSONReader();
        NetworkStructureInformation networkStructureInformation = jsonReader.getNetworkStructureInstance(json);
        this.networkName = networkStructureInformation.getNetworkName();
        this.instanceName = networkStructureInformation.getInstanceName();
        this.atomic = networkStructureInformation.isAtomic();
        this.subNetworks = networkStructureInformation.getSubNetworks();
    }


    public boolean equals(NetworkStructureInformation net) {
        boolean subnetEquality = true;

        if (this.subNetworks == null && net.getSubNetworks() == null) subnetEquality = true;
        else if ((this.subNetworks == null && net.getSubNetworks() != null) || (this.subNetworks != null && net.getSubNetworks() == null) || this.subNetworks.size() != net.getSubNetworks().size())
            subnetEquality = false;
        else {
            for (int i = 0; i < subNetworks.size(); i++) {
                boolean eq = this.subNetworks.get(i).equals(net.getSubNetworks().get(i));
                if (!eq) {
                    subnetEquality = false;
                    break;
                }
            }
        }
        return this.networkName.equals(net.networkName) && this.atomic == net.atomic && this.instanceName.equals(net.getInstanceName()) && subnetEquality;
    }

    public ArrayList<NetworkStructureInformation> getSubNetworks() {
        return subNetworks;
    }

    public boolean isAtomic() {
        return atomic;
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
}
