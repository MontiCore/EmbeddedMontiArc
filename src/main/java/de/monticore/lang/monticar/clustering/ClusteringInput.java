/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.clustering.Simulation.Edge;
import de.monticore.lang.monticar.clustering.helpers.ComponentHelper;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

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

    private List<Edge<Integer>> uniqueLabeledEdges;
    private boolean initUniqueLabeledEdges = false;


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
            labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(getSubcompsOrderedByName());
        }
        return new HashMap<>(labelsForSubcomps);
    }

    public List<Edge<Integer>> getUniqueLabeledEdges(){
        if(!initUniqueLabeledEdges){
            initUniqueLabeledEdges = true;
            // Get representative connectors(^= edges)
            uniqueLabeledEdges = getComponent().getSubComponentConnectors().stream()
                    .map(con -> new Edge<Integer>(getLabelsForSubcomps().get(con.getSourcePort().getComponentInstance().getFullName()), getLabelsForSubcomps().get(con.getTargetPort().getComponentInstance().getFullName())))
                    .distinct()
                    .collect(Collectors.toList());
        }

        return new ArrayList<>(uniqueLabeledEdges);
    }

    public EMAComponentInstanceSymbol getSubcompForLabel(int label){
        if(!initLabelsForSubcomps){
            getLabelsForSubcomps();
        }

        EMAComponentInstanceSymbol res = subcompsOrderedByName.get(label);
        Integer curLabel = labelsForSubcomps.get(res.getFullName());
        if(label != curLabel){
            Log.error("Internal label mixup. " + res.getFullName() + " has labels: " + label + ", " + curLabel);
        }

        return res;
    }

    private double[][] getCopyOf(double[][] old){
        double[][] res = new double[old.length][old[0].length];
        for (int i = 0; i < old.length; i++) {
            if (old[i].length >= 0) System.arraycopy(old[i], 0, res[i], 0, old[i].length);
        }
        return res;
    }



}
