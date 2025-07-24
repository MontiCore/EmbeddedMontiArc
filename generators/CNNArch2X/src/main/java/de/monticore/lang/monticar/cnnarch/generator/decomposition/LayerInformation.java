package de.monticore.lang.monticar.cnnarch.generator.decomposition;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import java.util.ArrayList;
import java.util.List;

public class LayerInformation {
    private String layerName;
    private ArrayList<String> parallelNames = null;
    private List<List<String>> parallelStreams = null;
    private LayerType layerType;
    private LayerInformation preceedingLayer = null;
    private LayerInformation succeedingLayer = null;
    private ArchitectureElementSymbol architectureElementSymbol = null;

    public LayerInformation(String layerName, LayerType layerType, ArchitectureElementSymbol architectureElementSymbol){
        this.layerName = layerName;
        this.layerType = layerType;
        this.architectureElementSymbol = architectureElementSymbol;
    }

    public LayerInformation(ArrayList<String> parallelNames, LayerType layerType, ArchitectureElementSymbol architectureElementSymbol){
        this.layerName = "parallel";
        this.parallelNames = parallelNames;
        this.layerType = layerType;
        this.architectureElementSymbol = architectureElementSymbol;
    }

    public LayerInformation(List<List<String>> parallelStreams, LayerType layerType, ArchitectureElementSymbol architectureElementSymbol){
        this.layerName = "parallel";
        this.parallelStreams = parallelStreams;
        this.layerType = layerType;
        this.architectureElementSymbol = architectureElementSymbol;
    }

    public void setLayerName(String layerName) { this.layerName = layerName; }

    public List<List<String>> getParallelStreams(){ return this.parallelStreams; }

    public LayerInformation getPreceedingLayer() {
        return preceedingLayer;
    }

    public void setPreceedingLayer(LayerInformation preceedingLayer){
        this.preceedingLayer = preceedingLayer;
    }

    public LayerInformation getSucceedingLayer() {
        return succeedingLayer;
    }

    public void setSucceedingLayer(LayerInformation succeedingLayer){
        this.succeedingLayer = succeedingLayer;
    }

    public String getLayerName(){
        return this.layerName;
    }

    public LayerType getLayerType(){
        return this.layerType;
    }

    public ArrayList<String> getParallelNames(){
        return this.parallelNames;
    }

    public boolean parallelNamesContain(String name){
        name = name.replace("_","");
        for (String para : this.parallelNames){
            para = para.replace("_","");
            if (para.equals(name)) return true;
        }
        return false;
    }

    public boolean isParallel() {
        return this.layerType == LayerType.PARALLEL_INPUT || this.layerType == LayerType.PARALLEL_OUTPUT || this.layerType == LayerType.PARALLEL_DEFAULT;
    }

    public boolean isInputLayer(){
        return this.layerType == LayerType.INPUT || this.layerType == LayerType.PARALLEL_INPUT; //&& this.preceedingLayer == null && this.succeedingLayer != null;
    }

    public boolean isOutputLayer(){
        return this.layerType == LayerType.OUTPUT || this.layerType == LayerType.PARALLEL_OUTPUT; //&& this.preceedingLayer != null && this.succeedingLayer == null;
    }

    public boolean isDefaultLayer(){
        return this.layerType == LayerType.DEFAULT; //&& this.preceedingLayer != null && this.succeedingLayer != null;
    }

    public boolean isFunctionLayer(){
        return this.layerType == LayerType.FUNCTION; //&& this.preceedingLayer != null && this.succeedingLayer != null;
    }
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Layer Name: ").append(layerName).append(", Layer Type: ").append(layerType);
        if (parallelStreams != null) {
            sb.append(", Parallel Streams: ");
            for (List<String> list : parallelStreams) {
                sb.append("[");
                for (int i = 0; i < list.size(); i++) {
                    sb.append(list.get(i));
                    if (i < list.size() - 1) {
                        sb.append(", ");
                    }
                }
                sb.append("]");
            }
        }
        return sb.toString();
    }
}
