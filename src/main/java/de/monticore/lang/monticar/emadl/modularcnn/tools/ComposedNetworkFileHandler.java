/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.tools;

import de.monticore.lang.monticar.emadl.modularcnn.compositions.ComponentInformation;
import de.monticore.lang.monticar.emadl.modularcnn.compositions.NetworkStructureInformation;

import java.io.*;
import java.util.ArrayList;

public class ComposedNetworkFileHandler {
    private String composedNetworksFilePath;
    public ComposedNetworkFileHandler(String composedNetworksFilePath){
        if (composedNetworksFilePath == null || composedNetworksFilePath.equals("")){
            this.composedNetworksFilePath = "ComposedNetworks";
        } else {
            this.composedNetworksFilePath = composedNetworksFilePath ;
        }
    }


    public void documentNetworkInFile(ComponentInformation componentInformation){
        writeNetworkFile(componentInformation, composedNetworksFilePath);
    }

    public ArrayList<NetworkStructureInformation> fetchKnownNetworksFromFile(){
        ArrayList<NetworkStructureInformation> knownNetworks = new ArrayList<>();
        ArrayList<String> knownNetworksJSONs = readNetworkFile(composedNetworksFilePath);

        if (knownNetworksJSONs == null) {
            knownNetworksJSONs = new ArrayList<>();
        }

        for (String jsonString : knownNetworksJSONs){
            knownNetworks.add(new NetworkStructureInformation(jsonString));
        }

        return knownNetworks;
    }

    private void writeNetworkFile(ComponentInformation componentInformation, String composedNetworksFilePath){
        if (componentInformation == null) return;

        try {
            ArrayList<NetworkStructureInformation> knownNetworks = fetchKnownNetworksFromFile();

            boolean hit = false;
            for (int i = 0; i<knownNetworks.size(); i++){
                NetworkStructureInformation network = knownNetworks.get(i);
                if (network.printStructureJSON().equals(componentInformation.printNetworkStructureJSON())){
                    hit = true;
                }
            }

            if (!hit){
                NetworkStructureInformation newNet = new NetworkStructureInformation(componentInformation);
                knownNetworks.add(newNet);
            }

            if (knownNetworks.size() > 0){
                removeFile(composedNetworksFilePath);
                for (NetworkStructureInformation network: knownNetworks){
                    writeToFile(composedNetworksFilePath,network.printStructureJSON());
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private ArrayList<String> readNetworkFile(String composedNetworksFilePath){
        return readFromFile(composedNetworksFilePath);
    }

    private ArrayList<String> readFromFile(String path){
        ArrayList<String> lines = new ArrayList<>();
        BufferedReader reader;

        File file = new File(path);
        if (!file.exists()){
            return null;
        }

        try {
            reader = new BufferedReader(new FileReader(path));
            String line = reader.readLine();
            while (line != null) {
                line = line.replaceAll("\n", "");
                lines.add(line);
                line = reader.readLine();
            }
            reader.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return lines;
    }

    private void writeToFile(String path,String content) throws IOException {
        createFileIfNotExists(path);
        BufferedWriter writer = new BufferedWriter(new FileWriter(path,true));
        writer.append(content).append("\n");
        writer.close();
    }

    private void createFileIfNotExists(String path) throws IOException{
        File f = new File(path);

        if (f.exists()){
            return;
        } else {
            boolean success = f.createNewFile();
            if (!success) throw new IOException("Could not create file: " + path);
        }
    }

    private void removeFile(String path) throws IOException{
        if (path.equals("")) {
            return;
        }

        File file = new File(path);
        if (!file.exists()) {
            return;
        }

        boolean success = file.delete();
        if (!success) throw new IOException("File could not be deleted:" + file.toString());
    }
}