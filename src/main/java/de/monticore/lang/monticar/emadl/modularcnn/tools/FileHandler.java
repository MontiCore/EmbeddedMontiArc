/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn.tools;

import de.monticore.lang.monticar.emadl.modularcnn.composer.ComponentInformation;
import de.monticore.lang.monticar.emadl.modularcnn.composer.NetworkStructureInformation;

import java.io.*;
import java.util.ArrayList;

public class FileHandler {
    public FileHandler(){}

    public void documentNetwork(ComponentInformation componentInformation, String composedNetworksFilePath){
        writeNetworkFile(componentInformation, composedNetworksFilePath);
    }

    private void writeNetworkFile(ComponentInformation componentInformation, String composedNetworksFilePath){
        if (componentInformation == null) return;

        try {
            ArrayList<String> knownNetworksJSONs = readNetworkFile(composedNetworksFilePath);
            ArrayList<NetworkStructureInformation> knownNetworks = new ArrayList<>();

            for (String jsonString : knownNetworksJSONs){
                knownNetworks.add(new NetworkStructureInformation(jsonString));
            }

            boolean hit = false;
            for (int i = 0; i<knownNetworks.size();i++){
                NetworkStructureInformation network = knownNetworks.get(i);
                if (network.isSubNetOf(componentInformation)){
                    hit = true;
                    knownNetworks.set(i, new NetworkStructureInformation(componentInformation));
                } else if( network.isSuperNetOf(componentInformation)){
                    hit = true;
                }
            }

            if (!hit){
                NetworkStructureInformation newNet = new NetworkStructureInformation(componentInformation);
                knownNetworks.add(newNet);
            }

            if (knownNetworks == null){
                removeFiles(composedNetworksFilePath);
                for (NetworkStructureInformation network: knownNetworks){
                    writeToFile(composedNetworksFilePath,network.printStructureJSON());
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public ArrayList<String> readNetworkFile(String composedNetworksFilePath){
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
        writer.append(content + "\n");
        writer.close();
    }

    public void createFileIfNotExists(String path) throws IOException{
        File f = new File(path);

        if (f.exists()){
            return;
        } else {
            boolean success = f.createNewFile();
            if (!success) throw new IOException("Could not create file: " + path);
        }
    }

    public void removeFiles(String path) throws IOException{
        if (path.equals("")) {
            return;
        }

        File files = new File(path);
        if (!files.exists()) {
            return;
        }

        for (File file: files.listFiles()){
            if (file.isDirectory())removeFiles(file.getPath());

            boolean success = file.delete();
            if (!success) throw new IOException("File could not be deleted:" + file.toString());
        }
    }
}
