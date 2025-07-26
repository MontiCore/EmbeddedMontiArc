/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.algorithms;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.clustering.ClusteringInput;
import de.se_rwth.commons.logging.Log;
import net.sf.javaml.clustering.mcl.SparseMatrix;
import net.sf.javaml.core.Dataset;
import net.sf.javaml.core.DefaultDataset;
import net.sf.javaml.core.DenseInstance;
import net.sf.javaml.core.Instance;

import java.util.*;

// markov mcl clusterer product implementation
public class MarkovClusteringAlgorithm implements ClusteringAlgorithm {
    private Object[] args;

    public void setArgs(Object[] args) {
        this.args = args;
    }

    @Override
    public Object[] getArgs() {
        return args;
    }


    private static Dataset[] getClustering(Dataset data, SparseMatrix matrix) {
        int[] sparseMatrixSize = matrix.getSize();
        int attractors = 0;

        // just for testing/debugging purposes
/*
        for(int i = 0; i < sparseMatrixSize[0]; ++i) {
            double val = matrix.get(i, i);
            if (val != 0.0D) {
                ++attractors;
            }
        }
*/

        Vector<Vector<Instance>> finalClusters = new Vector();

        for(int i = 0; i < sparseMatrixSize[0]; ++i) {
            Vector<Instance> cluster = new Vector();
            double val = matrix.get(i, i);
            if (val >= 0.98D) {
                for(int j = 0; j < sparseMatrixSize[0]; ++j) {
                    double value = matrix.get(j, i);
                    if (value != 0.0D) {
                        cluster.add(data.instance(j));
                    }
                }

                finalClusters.add(cluster);
            }
        }

        Dataset[] output = new Dataset[finalClusters.size()];

        int i;
        for(i = 0; i < finalClusters.size(); ++i) {
            output[i] = new DefaultDataset();
        }

        for(i = 0; i < finalClusters.size(); ++i) {
            new Vector();
            Vector<Instance> getCluster = (Vector)finalClusters.get(i);

            for(int j = 0; j < getCluster.size(); ++j) {
                output[i].add((Instance)getCluster.get(j));
            }
        }

        return output;
    }

    @Override
    public List<Set<EMAComponentInstanceSymbol>> cluster(ClusteringInput clusteringInput, Object... args) {

        List<Set<EMAComponentInstanceSymbol>> res = new ArrayList<>();

        // params
        Double maxResidual= null;
        Double gammaExp= null;
        Double loopGain= null;
        Double zeroMax= null;

        // find mandatory params
        Map<MarkovClusteringBuilder.MarkovParameters, Boolean> mandatoryParams = new HashMap<MarkovClusteringBuilder.MarkovParameters, Boolean>();
        MarkovClusteringBuilder.MarkovParameters[] markovParams = MarkovClusteringBuilder.MarkovParameters.values();
        for (MarkovClusteringBuilder.MarkovParameters param : markovParams) {
            // set all mandatory params to "unset"
            if (param.isMandatory()) mandatoryParams.put(param, false);
        }

        // Handle (optional) params for MarkovClustering.
        // Params come as one or multiple key-value-pairs in the optional varargs array for this method,
        // with key as a string (containing the name of the parameter to pass thru to the mcl clusterer) followed by its value as an object
        MarkovClusteringBuilder.MarkovParameters key;
        Object value;
        int v = 0;
        while (v < args.length) {
            if (args[v] instanceof MarkovClusteringBuilder.MarkovParameters) {
                key = (MarkovClusteringBuilder.MarkovParameters)args[v];
                if (v+1 < args.length) {
                    value = args[v + 1];
                    switch (key) {
                        case MARKOV_MAX_RESIDUAL:
                            if (value instanceof Double) {
                                maxResidual= (Double) value;
                            }
                            break;
                        case MARKOV_GAMMA_EXP:
                            if (value instanceof Double) {
                                gammaExp= (Double) value;
                            }
                            break;
                        case MARKOV_LOOP_GAIN:
                            if (value instanceof Double) {
                                loopGain= (Double) value;
                            }
                            break;
                        case MARKOV_ZERO_MAX:
                            if (value instanceof Double) {
                                zeroMax= (Double) value;
                            }
                            break;
                    }
                    // set mandatory param to "set"
                    if (key.isMandatory()) mandatoryParams.replace(key, true);
                }
            }
            v = v + 2;
        }

        // are all mandatory params set?
        boolean error= false;
        Iterator iterator = mandatoryParams.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry param = (Map.Entry) iterator.next();
            if (!(Boolean)param.getValue()) error= true;
        }

        if (error) {
            Log.error("MarkovClusteringAlgorithm: Mandatory parameter(s) missing!");
        } else {
            double[][] adjMatrix = clusteringInput.getAdjacencyMatrix();

            // |nodes| instances of data with one attribute denoting the node order
            Dataset original_ds= new DefaultDataset();
            for (int i=0; i<adjMatrix[0].length; i++) {
                original_ds.add(new DenseInstance(new double[]{i}));
            }
            MarkovClusteringBuilder builder = new MarkovClusteringBuilder(AutomaticClusteringHelper.weightedAdjacencyMatrix2transitionMatrix(adjMatrix));
            if (maxResidual != null) builder.setMaxResidual(maxResidual);
            if (gammaExp != null) builder.setGammaExp(gammaExp);
            if (loopGain != null) builder.setLoopGain(loopGain);
            if (zeroMax != null) builder.setZeroMax(zeroMax);
            SparseMatrix matrix = builder.build();

            Dataset[] clustered_ds= getClustering(original_ds, matrix);

            // interpret clustering for Monti
            int data_point;
            int[] labels = new int[original_ds.size()];
            for (int cluster=0; cluster < clustered_ds.length; cluster++) {
                for (int instance=0; instance < clustered_ds[cluster].size(); instance++) {
                    data_point= clustered_ds[cluster].instance(instance).get(0).intValue();
                    labels[data_point]= cluster;
                }
            }

            for (int i = 0; i < clustered_ds.length; i++) {
                res.add(new HashSet<>());
            }

            clusteringInput.getSubcompsOrderedByName().forEach(sc -> {
                int curClusterLabel = labels[clusteringInput.getLabelsForSubcomps().get(sc.getFullName())];
                res.get(curClusterLabel).add(sc);
            });
        }

        return res;
    }
}
