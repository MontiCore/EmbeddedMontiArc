package de.monticore.lang.monticar.emadl.modularcnn.composer;

import de.monticore.lang.monticar.emadl.modularcnn.tools.JSONBuilder;

import java.util.ArrayList;

public class NetworkStructureInformation {
    private ArrayList<NetworkStructureInformation> subNetworks = new ArrayList<>();
    private boolean atomic;
    private String networkName;

    public NetworkStructureInformation(ComponentInformation componentInformation) {
        this.networkName = componentInformation.getComponentName();
        ArrayList<ComponentInformation> subComponentsInformation = componentInformation.getSubComponentsInformation();

        if (subComponentsInformation == null || subComponentsInformation.size() == 0){
            this.atomic = true;
        } else {
            for (ComponentInformation comp: subComponentsInformation) {
                subNetworks.add(comp.analyzeNetworkStructure());
            }
        }
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

    public String atomicInfo() {
        if (atomic) return "atomic";
        return "composed";
    }

    public String printStructure(){
        String structure = this.getNetworkName() + " | " + this.atomicInfo();

        if (this.getSubNetworks().size() > 0){
            structure += " : ( ";

            for (NetworkStructureInformation subNetworkStructureInformation : this.getSubNetworks()){
                structure += subNetworkStructureInformation.printStructure();
                if (! (this.getSubNetworks().get(this.getSubNetworks().size()-1).equals(subNetworkStructureInformation))){
                    structure += " , ";
                }
            }
            structure += " )";
        }
        return structure;

    }

    public String printStructureJSON(){
        if (this.networkName.equals("") || this.networkName == null){
            return  "";
        }

        String jsonContent = "";
        jsonContent += JSONBuilder.JSONEntry("name",this.networkName,false);
        jsonContent += JSONBuilder.JSONEntry("atomic",this.atomic,false);

        ArrayList<String> arrayContents = new ArrayList<>();
        for (NetworkStructureInformation networkStructureInformation : this.getSubNetworks()){
            // TODO: Fix JSON printing -> Dual array brackets ( [[]] ) and whitespace in recursive printed arrays
            String subNetContent = networkStructureInformation.printStructureJSON();
            arrayContents.add(subNetContent);
        }

        jsonContent += JSONBuilder.JSONArray("subNetworks", arrayContents,true);
        String json = JSONBuilder.JSONObject(jsonContent,true);

        return json;
    }

    public boolean isSubNet(ComponentInformation componentInformation){
        return true;
    }

    public boolean isSubNet(NetworkStructureInformation networkStructureInformation){
        return true;
    }
}
