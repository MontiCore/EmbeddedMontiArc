package de.monticore.lang.monticar.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.clustering.helpers.ComponentHelper;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ClusteringInput {
    private EMAComponentInstanceSymbol instanceSymbol;

    private double[][] adjacencyMatrix;
    private boolean initAdjMatrix = false;

    private double[][] distanceMatrix;
    private boolean initDistMatrix = false;

    private List<EMAComponentInstanceSymbol> subcompsOrderedByName;
    private boolean initSubcomposOrderedByName = false;

    private Map<String, Integer> labelsForSubcomps;
    private boolean initLabelsForSubcomps = false;


    public ClusteringInput(EMAComponentInstanceSymbol instanceSymbol) {
        this.instanceSymbol = instanceSymbol;
    }

    public EMAComponentInstanceSymbol getComponent() {
        return instanceSymbol;
    }

    public double[][] getAdjacencyMatrix(){
        if(!initAdjMatrix){
            initAdjMatrix = true;
            adjacencyMatrix = AutomaticClusteringHelper.guaranteedConnectedAdjacencyMatrix(getSubcompsOrderedByName(), ComponentHelper.getInnerConnectors(instanceSymbol),getLabelsForSubcomps());

        }
        return getCopyOf(adjacencyMatrix);
    }

    public double[][] getDistanceMatrix(){
        if(!initDistMatrix){
            initDistMatrix = true;
            distanceMatrix = AutomaticClusteringHelper.getDistanceMatrix(getAdjacencyMatrix());

        }
        return getCopyOf(distanceMatrix);
    }

    public List<EMAComponentInstanceSymbol> getSubcompsOrderedByName(){
        if(!initSubcomposOrderedByName){
            initSubcomposOrderedByName = true;
            subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(instanceSymbol);
        }
        return new ArrayList<>(subcompsOrderedByName);
    }

    public Map<String, Integer> getLabelsForSubcomps(){
        if(!initLabelsForSubcomps){
            initLabelsForSubcomps = true;
            labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName);
        }
        return new HashMap<>(labelsForSubcomps);
    }

    private double[][] getCopyOf(double[][] old){
        double[][] res = new double[old.length][old[0].length];
        for (int i = 0; i < old.length; i++) {
            if (old[i].length >= 0) System.arraycopy(old[i], 0, res[i], 0, old[i].length);
        }
        return res;
    }



}
