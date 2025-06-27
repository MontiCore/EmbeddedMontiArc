/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.algorithms;

import net.sf.javaml.clustering.mcl.MarkovClustering;
import net.sf.javaml.clustering.mcl.SparseMatrix;

public class MarkovClusteringBuilder {

    private double[][] data;    // expected: transition matrix
    private double maxResidual = 0.001;
    private double gammaExp = 2.0;
    private double loopGain = 0.;
    private double zeroMax = 0.001;

    // parameter list, true if mandatory
    public enum MarkovParameters {
        MARKOV_MAX_RESIDUAL(false),
        MARKOV_GAMMA_EXP(false),
        MARKOV_LOOP_GAIN(false),
        MARKOV_ZERO_MAX(false);

        private Boolean mandatory;

        MarkovParameters(Boolean mandatory) {
            this.mandatory = mandatory;
        }

        public Boolean isMandatory() {
            return this.mandatory;
        }
    }

    public MarkovClusteringBuilder(double[][] data) {
        this.data = data;
    }


    public MarkovClusteringBuilder setData(double[][] data) {
        this.data = data;
        return this;
    }

    public MarkovClusteringBuilder setMaxResidual(double maxResidual) {
        this.maxResidual = maxResidual;
        return this;
    }

    public MarkovClusteringBuilder setGammaExp(double gammaExp) {
        this.gammaExp = gammaExp;
        return this;
    }

    public MarkovClusteringBuilder setLoopGain(double loopGain) {
        this.loopGain = loopGain;
        return this;
    }

    public MarkovClusteringBuilder setZeroMax(double zeroMax) {
        this.zeroMax = zeroMax;
        return this;
    }

    public SparseMatrix build() {
        SparseMatrix matrix = null;

        MarkovClustering mc = new MarkovClustering();

        SparseMatrix smatrix = new SparseMatrix(data);
        matrix = mc.run(smatrix, this.maxResidual, this.gammaExp, this.loopGain, this.zeroMax);

        return matrix;
    }

}
