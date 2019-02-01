package de.monticore.lang.monticar.generator.middleware.cli.algorithms;

import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.DBSCANClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.DBSCANClusteringBuilder;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class DBScanCliParameters extends AlgorithmCliParameters {

    private Integer min_pts;
    private Double radius;

    public DBScanCliParameters() {
    }

    @Override
    public String getName() {
        return TYPE_DBSCAN;
    }

    @Override
    public ClusteringAlgorithm asClustringAlgorithm() {
        DBSCANClusteringAlgorithm dbscanClusteringAlgorithm = new DBSCANClusteringAlgorithm();
        dbscanClusteringAlgorithm.setArgs(asAlgorithmArgs());
        return dbscanClusteringAlgorithm;
    }

    @Override
    public List<Object> asAlgorithmArgs() {
        List<Object> res = new ArrayList<>();
        if(!isValid()){
            Log.error("DBScanCliParameters: The min_pts or radius parameters are mandatory but at least one is unset!");
            return res;
        }

        res.add(DBSCANClusteringBuilder.DBSCANParameters.DBSCAN_MIN_PTS);
        res.add(min_pts);
        res.add(DBSCANClusteringBuilder.DBSCANParameters.DBSCAN_RADIUS);
        res.add(radius);

        return res;
    }

    @Override
    public boolean isValid() {
        return min_pts != null && radius != null;
    }

    public Optional<Integer> getMinPts() {
        return Optional.ofNullable(min_pts);
    }

    public Optional<Double> getRadius() {
        return Optional.ofNullable(radius);
    }

    public void setMinPts(Integer min_pts) {
        this.min_pts = min_pts;
    }

    public void setRadius(Double radius) {
        this.radius = radius;
    }
}
