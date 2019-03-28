package de.monticore.lang.monticar.generator.middleware.clustering.algorithms;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.AutomaticClusteringHelper;
import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.helpers.ComponentHelper;
import de.se_rwth.commons.logging.Log;
import smile.clustering.DBSCAN;

import java.util.*;

// DBSCAN clusterer product implementation
public class DBSCANClusteringAlgorithm implements ClusteringAlgorithm {
    @Override
    public List<Set<EMAComponentInstanceSymbol>> cluster(EMAComponentInstanceSymbol component, Object... args) {

        List<Set<EMAComponentInstanceSymbol>> res = new ArrayList<>();

        // params
        Integer minPts= null;
        Double radius= null;

        // find mandatory params
        Map<DBSCANClusteringBuilder.DBSCANParameters, Boolean> mandatoryParams = new HashMap<DBSCANClusteringBuilder.DBSCANParameters, Boolean>();
        DBSCANClusteringBuilder.DBSCANParameters[] dbscanParams = DBSCANClusteringBuilder.DBSCANParameters.values();
        for (DBSCANClusteringBuilder.DBSCANParameters param : dbscanParams) {
            // set all mandatory params to "unset"
            if (param.isMandatory()) mandatoryParams.put(param, false);
        }

        // Handle (optional) params for DBSCANClustering.
        // Params come as one or multiple key-value-pairs in the optional varargs array for this method,
        // with key as a string (containing the name of the parameter to pass thru to the clusterer) followed by its value as an object
        DBSCANClusteringBuilder.DBSCANParameters key;
        Object value;
        int v = 0;
        while (v < args.length) {
            if (args[v] instanceof DBSCANClusteringBuilder.DBSCANParameters) {
                key = (DBSCANClusteringBuilder.DBSCANParameters)args[v];
                if (v+1 < args.length) {
                    value = args[v + 1];
                    switch (key) {
                        case DBSCAN_MIN_PTS:
                            if (value instanceof Integer) {
                                minPts= (Integer) value;
                            }
                            break;
                        case DBSCAN_RADIUS:
                            if (value instanceof Double) {
                                radius= (Double) value;
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
            Log.error("DBSCANClusteringAlgorithm: Mandatory parameter(s) missing!");
        } else {
            List<EMAComponentInstanceSymbol> subcompsOrderedByName = ComponentHelper.getSubcompsOrderedByName(component);
            Map<String, Integer> labelsForSubcomps = ComponentHelper.getLabelsForSubcomps(subcompsOrderedByName);
            double[][] adjMatrix = AutomaticClusteringHelper.createAdjacencyMatrix(subcompsOrderedByName,
                    ComponentHelper.getInnerConnectors(component),
                    labelsForSubcomps);

            DBSCAN clustering;
            DBSCANClusteringBuilder builder = new DBSCANClusteringBuilder(adjMatrix, minPts, radius);
            clustering = builder.build();

            int[] labels = clustering.getClusterLabel();

            for (int i = 0; i < clustering.getNumClusters(); i++) {
                res.add(new HashSet<>());
            }

            subcompsOrderedByName.forEach(sc -> {
                int curClusterLabel = labels[labelsForSubcomps.get(sc.getFullName())];
                res.get(curClusterLabel).add(sc);
            });
        }

        return res;
    }
}
