/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.algorithms;

import smile.math.distance.Metric;

import java.io.Serializable;

public class DBSCANDistance implements Metric<double[]>, Serializable {
    private static final long serialVersionUID = 1L;
    private double[][] weightedAdjacencyMatrix = null;

    public DBSCANDistance(double[][] weightedAdjacencyMatrix) {
        this.weightedAdjacencyMatrix = weightedAdjacencyMatrix;
    }

    public String toString() {
        return String.format("DBSCAN distance");
    }

    public double d(double[] x, double[] y) {
        System.out.println((int)x[0] + ", " + (int)y[0] + ": " + weightedAdjacencyMatrix[(int)x[0]][(int)y[0]]);
        if (x.length != y.length) {
            throw new IllegalArgumentException(String.format("Arrays have different length: x[%d], y[%d]", x.length, y.length));
        } else {
            double ret= weightedAdjacencyMatrix[(int)x[0]][(int)y[0]];
            return ret > 0 ? ret : Double.MAX_VALUE;    // set zeros in adj. matrix to infinity
        }
    }
}
