/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.algorithms;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.clustering.ClusteringInput;
import de.se_rwth.commons.logging.Log;
import smile.clustering.SpectralClustering;

import java.util.*;

// spectral clusterer product implementation
public class SpectralClusteringAlgorithm implements ClusteringAlgorithm {
    private Object[] args;

    public void setArgs(Object[] args) {
        this.args = args;
    }

    @Override
    public Object[] getArgs() {
        return args;
    }

    @Override
    public List<Set<EMAComponentInstanceSymbol>> cluster(ClusteringInput clusteringInput, Object... args) {

        List<Set<EMAComponentInstanceSymbol>> res = new ArrayList<>();

        // params
        Integer numClusters= null;
        Integer l= null;
        Double sigma= null;

        // find mandatory params
        Map<SpectralClusteringBuilder.SpectralParameters, Boolean> mandatoryParams = new HashMap<SpectralClusteringBuilder.SpectralParameters, Boolean>();
        SpectralClusteringBuilder.SpectralParameters[] spectralParams = SpectralClusteringBuilder.SpectralParameters.values();
        for (SpectralClusteringBuilder.SpectralParameters param : spectralParams) {
            // set all mandatory params to "unset"
            if (param.isMandatory()) mandatoryParams.put(param, false);
        }

        // Handle (optional) params for SpectralClustering.
        // Params come as one or multiple key-value-pairs in the optional varargs array for this method,
        // with key as a string (containing the name of the parameter to pass thru to the spectral clusterer) followed by its value as an object
        SpectralClusteringBuilder.SpectralParameters key;
        Object value;
        int v = 0;
        while (v < args.length) {
            if (args[v] instanceof SpectralClusteringBuilder.SpectralParameters) {
                key = (SpectralClusteringBuilder.SpectralParameters)args[v];
                if (v+1 < args.length) {
                    value = args[v + 1];
                    switch (key) {
                        case SPECTRAL_NUM_CLUSTERS:
                            if (value instanceof Integer) {
                                numClusters= (Integer) value;
                            }
                            break;
                        case SPECTRAL_L:
                            if (value instanceof Integer) {
                                l= (Integer) value;
                            }
                            break;
                        case SPECTRAL_SIGMA:
                            if (value instanceof Double) {
                                sigma= (Double) value;
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
            Log.error("SpectralClusteringAlgorithm: Mandatory parameter(s) missing!");
        } else {
            double[][] adjMatrix = clusteringInput.getAdjacencyMatrix();

            SpectralClustering clustering;
            SpectralClusteringBuilder builder = new SpectralClusteringBuilder(adjMatrix, numClusters);
            if (l != null) builder.setL(l);
            if (sigma != null) builder.setSigma(sigma);
            clustering = builder.build();

            int[] labels = clustering.getClusterLabel();

            for (int i = 0; i < clustering.getNumClusters(); i++) {
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
