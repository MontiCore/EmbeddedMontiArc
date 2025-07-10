/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.algorithms;

import smile.clustering.SpectralClustering;

public class SpectralClusteringBuilder {

    private double[][] data;    // expected: adjacency matrix
    private Integer k;
    private Integer l;
    private Double sigma;

    // parameter list, true if mandatory
    public enum SpectralParameters {
        SPECTRAL_NUM_CLUSTERS(true),
        SPECTRAL_L(false),
        SPECTRAL_SIGMA(false);

        private Boolean mandatory;

        SpectralParameters(Boolean mandatory) {
            this.mandatory = mandatory;
        }

        public Boolean isMandatory() {
            return this.mandatory;
        }
    }

    public SpectralClusteringBuilder(double[][] data, int k) {
        this.data = data;
        this.k = k;
    }


    public SpectralClusteringBuilder setData(double[][] data) {
        this.data = data;
        return this;
    }

    public SpectralClusteringBuilder setK(int k) {
        this.k = k;
        return this;
    }

    public SpectralClusteringBuilder setL(int l) {
        this.l = l;
        return this;
    }

    public SpectralClusteringBuilder setSigma(double sigma) {
        this.sigma = sigma;
        return this;
    }


    public SpectralClustering build() {
        SpectralClustering sc;

        if (this.l != null && this.sigma != null) {
            sc = new SpectralClustering(data, k, l, sigma);
        } else {
            if (this.sigma != null) {
                sc = new SpectralClustering(data, k, sigma);
            } else
                sc = new SpectralClustering(data, k);
        }

        return sc;
    }

}
