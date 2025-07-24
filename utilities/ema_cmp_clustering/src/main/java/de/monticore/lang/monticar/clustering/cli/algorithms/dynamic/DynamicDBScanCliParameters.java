/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms.dynamic;

import de.monticore.lang.monticar.clustering.cli.algorithms.AlgorithmCliParameters;
import de.monticore.lang.monticar.clustering.cli.algorithms.DBScanCliParameters;

import java.util.ArrayList;
import java.util.List;

public class DynamicDBScanCliParameters extends DynamicAlgorithmCliParameters {
    private DynamicParameter min_pts;
    private DynamicParameter radius;

    @Override
    public List<AlgorithmCliParameters> getAll() {
        List<AlgorithmCliParameters> res = new ArrayList<>();
        for(Integer pts : min_pts.getAllAsInt()){
            for(Double rad: radius.getAll()){
                res.add(new DBScanCliParameters(pts, rad));
            }
        }
        return res;
    }

    @Override
    public boolean isValid() {
        return min_pts != null && min_pts.isValid() && radius != null && radius.isValid();
    }

    @Override
    public String getName() {
        return AlgorithmCliParameters.TYPE_DBSCAN;
    }
}
