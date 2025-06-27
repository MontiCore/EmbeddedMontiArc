package de.monticore.lang.monticar.cnnarch.gluongenerator.decomposition;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import de.monticore.lang.monticar.cnnarch.generator.decomposition.BackendDecomposer;
import de.monticore.lang.monticar.cnnarch.generator.decomposition.LayerInformation;
import de.monticore.lang.monticar.cnnarch.generator.decomposition.NetworkStructure;
import de.se_rwth.commons.logging.Log;

import java.io.*;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class GluonDecomposer implements BackendDecomposer {

    List<LayerSubstitute> allowedLayerSubstitutes = AllSubstitutes.getAllSubstitutes();
    String pythonPath = "";
    String pythonTool = "target/GluonParameterSplitter.py";

    public GluonDecomposer(String pythonPath) {
        this.pythonPath = pythonPath;
        rebuildPythonScript();
    }

    public void listFilesUsingJavaIO(String dir) {
        Log.warn(Stream.of(new File(dir).listFiles())
                .filter(file -> !file.isDirectory())
                .map(File::getName)
                .collect(Collectors.toSet()).toString());
    }

    @Override
    public void decomposeNetwork(String modelPath, NetworkStructure networkStructure) {
        ArrayList<File> fileList = scanForFiles(modelPath, networkStructure);

        File networkJsonFile = null;
        File paramsFile = null;


        for (File file : fileList) {
            String fileName = file.getName();
            if (fileName.contains("newest-symbol.json")) networkJsonFile = file;
            else if (fileName.contains("newest-0000.params")) paramsFile = file;
        }

        splitComposedNetwork(modelPath, networkStructure, networkJsonFile, paramsFile);
    }

    private void splitComposedNetwork(String modelPath, NetworkStructure composedNetworkStructure, File networkFile, File paramsFile) {

        if (networkFile == null || paramsFile == null) return;
        ArrayList<GluonRepresentation> splitGluonNets = splitNetworkJsonFile(modelPath, composedNetworkStructure, networkFile);

        for (GluonRepresentation gluonNet : splitGluonNets) {
            String jsonContent = mapToJson(gluonNet.getGluonJsonRepresentation());
            ArrayList<String> parameterLayers = gluonNet.getParameterLayerCandidates();

            String decomposedNetDirectory = modelPath + composedNetworkStructure.getComponentName() + "." + gluonNet.getNetworkName() + "_decomposed";
            File directory = new File(decomposedNetDirectory);

            if (!directory.exists()) {
                directory.mkdir();
            }

            String decomposedFileName = "model_" + gluonNet.getNetworkName() + "_decomposed";

            String decomposedNetPath = decomposedNetDirectory + "/" + decomposedFileName + "-old-symbol.json";
            writeFile(decomposedNetPath, jsonContent);
            //Log.warn("Running Decomposition for: " + gluonNet.getNetworkName());
            generateNewParamsFileWithPython(decomposedNetDirectory, decomposedFileName, decomposedNetPath, gluonNet.getParameterLayerCandidates(), paramsFile, networkFile, gluonNet);
        }
    }

    private void generateNewParamsFileWithPython(String networkDirectory, String networkName, String decomposedNetFullPath,
                                                 ArrayList<String> parameterLayers, File originalParamFile, File originalNetworkFile, GluonRepresentation gluonNet) {
        ArrayList<String> pythonCall = new ArrayList<>();
        pythonCall.add(pythonPath);
        pythonCall.add(pythonTool);

        pythonCall.add("-in");
        pythonCall.add(gluonNet.getNetworkStructure().getNetworkLayers().get(0).getLayerName());

        pythonCall.add("-mp");
        pythonCall.add(decomposedNetFullPath);

        pythonCall.add("-pp");
        pythonCall.add(originalParamFile.getPath());

        pythonCall.add("-nmd");
        pythonCall.add(networkDirectory);

        pythonCall.add("-nmn");
        pythonCall.add(networkName);

        pythonCall.add("-onp");
        pythonCall.add(originalNetworkFile.getPath());

        StringBuilder shapeStrBuilder = new StringBuilder();
        for (Map.Entry<String, ArrayList<Integer>> entry : gluonNet.getNetworkStructure().getInputPortsDim().entrySet()) {
            shapeStrBuilder.append(entry.getKey());
            shapeStrBuilder.append(":");
            for (int i = 0; i < entry.getValue().size(); i++) {
                shapeStrBuilder.append(entry.getValue().get(i));
                if (i < entry.getValue().size() - 1) {
                    shapeStrBuilder.append(",");
                }
            }
            shapeStrBuilder.append(";");
        }
        if (shapeStrBuilder.length() > 0) {
            shapeStrBuilder.setLength(shapeStrBuilder.length() - 1);
        }

        pythonCall.add("-shape");
        pythonCall.add(shapeStrBuilder.toString());

        StringBuilder layerList = new StringBuilder();
        for (int i = 0; i < parameterLayers.size(); i++) {
            String layer = parameterLayers.get(i);
            layerList.append(layer);

            if (i != parameterLayers.size() - 1) layerList.append(",");
        }

        pythonCall.add("-pl");
        pythonCall.add(layerList.toString());

        ProcessBuilder pb = new ProcessBuilder(pythonCall).inheritIO();
        int exitCode = 0;

        try {
            Process p = pb.start();

            exitCode = p.waitFor();
        } catch (IOException e) {
            String errMsg = "IOException when writing new gluon param file with exit code: " + Integer.toString(exitCode);
            Log.error(errMsg);
            throw new RuntimeException(errMsg);
        } catch (InterruptedException e) {
            String errMsg = "Interrupted Exception when writing new gluon param file with exit code: " + Integer.toString(exitCode);
            Log.error(errMsg);
            throw new RuntimeException(errMsg);
        }

        if (exitCode != 0) {
            String errMsg = "Exit code that is not 0. Exit code: " + Integer.toString(exitCode);
            Log.error(errMsg);
            throw new RuntimeException(errMsg);
        } else {
            Log.info("Wrote new params file for network " + networkName, "DECOMPOSITION");
        }
    }

    private ArrayList<GluonRepresentation> splitNetworkJsonFile(String modelPath, NetworkStructure networkStructure, File file) {
        ArrayList<GluonRepresentation> gluonNets = new ArrayList<>();

        String symbolJson = readFile(file.getPath());
        Map<String, Object> contentMap = jsonToMap(symbolJson);

        Map<String, Object> attributes = (Map<String, Object>) contentMap.get("attrs");

        ArrayList<Object> nodes = (ArrayList<Object>) contentMap.get("nodes");
        ListIterator<Object> nodeIterator = nodes.listIterator();

        ArrayList<NetworkStructure> subNets = networkStructure.getNetsToDecompose();

        int headSize = getHeadSize((Map<String, Object>) nodes.get(nodes.size() - 1));
        int nodeDifference = 0;
        int generatedNodesMalus = 0;

        for (int i = 0, subNetsSize = subNets.size(); i < subNetsSize; i++) {
            NetworkStructure currentNetwork = subNets.get(i);
            ArrayList<Map<String, Object>> currentNetworkNodes = new ArrayList<>();
            for (LayerInformation layer : currentNetwork.getNetworkLayers()) {
                switch (layer.getLayerType()) {
                    case INPUT:
                        generatedNodesMalus = handleInput(layer, nodeIterator, currentNetworkNodes, i==0, generatedNodesMalus);
                        break;
                    case DEFAULT:
                        handleDefault(layer, nodeIterator, currentNetworkNodes);
                        break;
                    case PARALLEL_DEFAULT:
                        handleParallel(layer, nodeIterator, currentNetworkNodes);
                        break;
                    case OUTPUT:
                        generatedNodesMalus = handleOutput(layer, nodeIterator, currentNetworkNodes, generatedNodesMalus, i == subNetsSize - 1);
                        break;
                    default:
                        Log.error("Undefined behaviour for Layer: " + layer);
                }
            }

            gluonNets.add(new GluonRepresentation(currentNetwork, currentNetworkNodes, attributes, nodeDifference, headSize));
            nodeDifference += currentNetworkNodes.size() - generatedNodesMalus;
            if (i == 0) nodeDifference--;
            Log.debug("All nodes for " + currentNetwork.getNetworkName() +" processed","GluonDecomposer");
        }

        return gluonNets;
    }
    private int getHeadSize(Map<String, Object> lastNode) {
        ArrayList<Object> lastInputs = (ArrayList<Object>) lastNode.get("inputs");
        ArrayList<Integer> lastInputIntList = (ArrayList<Integer>) lastInputs.get(0);
        return lastInputIntList.size() - 1;
    }

    private int handleOutput(LayerInformation layer, ListIterator<Object> nodeIterator, ArrayList<Map<String, Object>> currentNodes, int generatedNodesMalus, boolean isLast) {
        if (isLast) {
            if (layer.isParallel()) {
                handleParallel(layer, nodeIterator, currentNodes);
            }
            else {
                if (nodeIterator.hasNext()) {
                    Map<String, Object> node = (Map<String, Object>) nodeIterator.next();
                    if (node.get("op").equals("_copy") && ((String) node.get("name")).contains("identity")) {
                        currentNodes.add(node);
                    }
                }
                else Log.error("No more nodes to process but the output layer has not been reached.");

            }
        } else {
            currentNodes.add(buildOutput());
            generatedNodesMalus++;
        }
        return generatedNodesMalus;
    }

    private void handleParallel(LayerInformation layer, ListIterator<Object> nodeIterator, ArrayList<Map<String, Object>> currentNodes) {
        Log.debug("Processing nodes for: " + layer,"GluonDecomposer");

        String combining_op = layer.getSucceedingLayer().getLayerName();
        while (nodeIterator.hasNext()) {
            Map<String, Object> node = (Map<String, Object>) nodeIterator.next();
            String op = (String) node.get("op");
            // TODO possible check of min processing of nodes included in the parallel stream
            if (op.equals(combining_op) || checkLayerSubstitutesForMatch(combining_op, node)) {
                nodeIterator.previous();
                break;
            }
            Log.debug("Added node op: " + node.get("op")+ " name " + node.get("name"),"GluonDecomposer");
            currentNodes.add(node);
        }
        Log.debug("Finished processing nodes for: " + layer,"GluonDecomposer");
        Log.debug("-","GluonDecomposer");

    }

    private void handleDefault(LayerInformation layer, ListIterator<Object> nodeIterator, ArrayList<Map<String, Object>> currentNodes) {
        Log.debug("Processing nodes for: " + layer,"GluonDecomposer");
        while (nodeIterator.hasNext()) {
            Map<String, Object> node = (Map<String, Object>) nodeIterator.next();
            currentNodes.add(node);
            Log.debug("Added node op: " + node.get("op")+ " name " + node.get("name"),"GluonDecomposer");
            String op = (String) node.get("op");
            if (op.equals(layer.getLayerName()) || checkLayerSubstitutesForMatch(layer.getLayerName(), node)) {
                break;
            }
        }
        Log.debug("Finished processing nodes for: " + layer,"GluonDecomposer");
        Log.debug("-","GluonDecomposer");

    }

    private int handleInput(LayerInformation layer, ListIterator<Object> nodeIterator, ArrayList<Map<String, Object>> currentNodes, boolean isFirst, int generatedNodesMalus) {
        if (isFirst) {
            if (layer.isParallel()) {
                handleParallel(layer, nodeIterator, currentNodes);
            } else {
                if (nodeIterator.hasNext()){
                    Map<String, Object> node = (Map<String, Object>) nodeIterator.next();
                    if (node.get("op").equals("null") && node.get("name").equals(layer.getLayerName())) {
                        currentNodes.add(node);
                    }
                }
                else Log.error("No nodes found for layer: " + layer);

            }
        } else {
            currentNodes.add(buildInput(layer.getLayerName()));
            generatedNodesMalus++;
        }
        return generatedNodesMalus;
    }

    private boolean checkLayerSubstitutesForMatch(String originalLayer, Map<String, Object> node) {
        String op = (String) node.get("op");
        Map<String, String> attributes;

        if (node.containsKey("attrs") && node.get("attrs") instanceof Map) {
            attributes = (Map<String, String>) node.get("attrs");
        }
        else attributes = Collections.emptyMap();

        for (LayerSubstitute layerSubstitute : this.allowedLayerSubstitutes) {
            if (layerSubstitute.hasLayerSubstitute(originalLayer, op) &&
                    layerSubstitute.isCorrectMatch(op, attributes)) {
                return true;
            }
        }
        return false;
    }

    private ArrayList<File> scanForFiles(String modelPath, NetworkStructure composedNetworkStructure) {
        ArrayList<File> files = new ArrayList<>();
        String path = modelPath + "/" + composedNetworkStructure.getComponentName();
        File modelDirectory = new File(path);
        File[] fileList = modelDirectory.listFiles();

        if (fileList != null) {
            for (File file : fileList) {
                if (file.isFile()) {
                    files.add(file);
                }
            }
        }
        return files;
    }

    private void writeFile(String path, String content) {
        File file = new File(path);
        try {
            FileWriter fileWriter = new FileWriter(path);
            fileWriter.write(content);
            fileWriter.close();
        } catch (IOException e) {
            Log.error("Error while writing decomposition files for GLUON backend");
            e.printStackTrace();
        }
    }

    private String readFile(String path) {
        StringBuilder content = new StringBuilder();
        try {
            Scanner scanner = new Scanner(new File(path));

            while (scanner.hasNextLine()) {
                content.append(scanner.nextLine());
            }
            scanner.close();
        } catch (Exception e) {
            Log.error("Error while reading GLUON file");
            e.printStackTrace();
        }
        return content.toString();
    }

    private Map<String, Object> jsonToMap(String jsonContent) {
        ObjectMapper mapper = new ObjectMapper();
        Map<String, Object> contentMap = new HashMap<>();
        TypeReference<HashMap<String, Object>> typeReference = new TypeReference<HashMap<String, Object>>() {
        };

        try {
            contentMap = mapper.readValue(jsonContent, typeReference);
        } catch (Exception e) {
            e.printStackTrace();
            Log.error("Error while mapping json content to map");
        }

        return contentMap;
    }

    private String mapToJson(Map<String, Object> contentMap) {
        String jsonResult = "";
        ObjectMapper mapper = new ObjectMapper();
        try {
            jsonResult = mapper.writerWithDefaultPrettyPrinter().writeValueAsString(contentMap);
        } catch (Exception e) {
            e.printStackTrace();
            Log.error("Error while creating json string from map");
        }

        return jsonResult;
    }

    private Map<String, Object> buildInput(String inputName) {
        return buildIONode("null", inputName, new ArrayList<Object>());
    }

    private Map<String, Object> buildOutput() {
        return buildIONode("_copy", "identity_gen", new ArrayList<Object>());
    }

    private Map<String, Object> buildIONode(String op, String name, ArrayList<Object> inputs) {
        Map<String, Object> ioNode = new LinkedHashMap<String, Object>();
        ioNode.put("op", op);
        ioNode.put("name", name);
        ioNode.put("inputs", inputs);
        return ioNode;
    }

    private void rebuildPythonScript() {
        BufferedReader b = new BufferedReader(new InputStreamReader(this.getClass().getClassLoader().getResourceAsStream("GluonParameterSplitter.py")));
        String line = null;
        try {
            FileWriter fw = new FileWriter("target/GluonParameterSplitter.py");
            line = b.readLine();
            while (line != null) {
                fw.write(line + "\n");
                line = b.readLine();
            }
            fw.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}