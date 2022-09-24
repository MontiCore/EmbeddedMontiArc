package de.monticore.lang.monticar.emadl.modularcnn.tools;

import de.monticore.lang.monticar.emadl.modularcnn.composer.ComponentInformation;
import de.se_rwth.commons.logging.Log;

import java.io.*;
import java.util.ArrayList;

public class FileHandler {
    public FileHandler(){}

    public void writeNetworkFile(ComponentInformation componentInformation, String composedNetworksFilePath, boolean json){
        if (componentInformation == null) return;

        try {
            ArrayList<String> knownNetworks = readNetworkFile(composedNetworksFilePath);
            if (knownNetworks == null || !knownNetworks.contains(componentInformation.printNetworkStructure())){
                String content;
                if (json) {

                    content = componentInformation.printNetworkStructureJSON();
                }
                else content =  componentInformation.printNetworkStructure();

                writeToFile(composedNetworksFilePath,content);
            }
        } catch (Exception e) {
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
        File f = new File(path);

        if (!f.exists()){
            boolean success = f.createNewFile();
            if (!success) throw new IOException("Could not create file: " + path);
        }

        BufferedWriter writer = new BufferedWriter(new FileWriter(path,true));
        writer.append(content + "\n");
        writer.close();
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
