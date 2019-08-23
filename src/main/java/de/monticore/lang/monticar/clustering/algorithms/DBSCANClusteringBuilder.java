/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.algorithms;

import smile.clustering.DBSCAN;

public class DBSCANClusteringBuilder {
    private double[][] data;    // expected: (weighted) adjacency matrix
    private Integer minPts;
    private Double radius;

    // parameter list, true if mandatory
    public enum DBSCANParameters {
        DBSCAN_MIN_PTS(true),
        DBSCAN_RADIUS(true);

        private Boolean mandatory;

        DBSCANParameters(Boolean mandatory) {
            this.mandatory = mandatory;
        }

        public Boolean isMandatory() {
            return this.mandatory;
        }
    }

    public DBSCANClusteringBuilder(double[][] data, int minPts, double radius) {
        this.data = data;
        this.minPts = minPts;
        this.radius= radius;
    }


    public DBSCANClusteringBuilder setData(double[][] data) {
        this.data = data;
        return this;
    }

    public DBSCANClusteringBuilder setMinPts(int minPts) {
        this.minPts = minPts;
        return this;
    }

    public DBSCANClusteringBuilder setRadius(double radius) {
        this.radius = radius;
        return this;
    }


    public DBSCAN build() {
        DBSCAN dbc;

        // |nodes| instances of data with pseudo x,y coords. set to node no.
        double[][] pdata = new double[this.data.length][2];
        for (int i=0; i<pdata.length; i++) {
            pdata[i][0]= i;
            pdata[i][1]= i;
        }
        dbc = new DBSCAN(pdata, new DBSCANDistance(data), minPts, radius);

        return dbc;
    }

}
