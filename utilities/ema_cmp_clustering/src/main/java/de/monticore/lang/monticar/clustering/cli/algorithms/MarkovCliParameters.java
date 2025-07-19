/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.cli.algorithms;

import de.monticore.lang.monticar.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.clustering.algorithms.MarkovClusteringAlgorithm;
import de.monticore.lang.monticar.clustering.algorithms.MarkovClusteringBuilder;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MarkovCliParameters extends AlgorithmCliParameters {
    private Double max_residual;
    private Double gamma_exp;
    private Double loop_gain;
    private Double zero_max;

    public MarkovCliParameters() {
    }

    public MarkovCliParameters(Double max_residual, Double gamma_exp, Double loop_gain, Double zero_max) {
        this.max_residual = max_residual;
        this.gamma_exp = gamma_exp;
        this.loop_gain = loop_gain;
        this.zero_max = zero_max;
    }

    @Override
    public String getName() {
        return TYPE_MARKOV;
    }

    @Override
    public ClusteringAlgorithm asClusteringAlgorithm() {
        MarkovClusteringAlgorithm markovClusteringAlgorithm = new MarkovClusteringAlgorithm();
        markovClusteringAlgorithm.setArgs(asAlgorithmArgs());
        return markovClusteringAlgorithm;
    }

    @Override
    public Object[] asAlgorithmArgs() {
        List<Object> res = new ArrayList<>();
        if(max_residual != null){
            res.add(MarkovClusteringBuilder.MarkovParameters.MARKOV_MAX_RESIDUAL);
            res.add(max_residual);
        }
        if(gamma_exp != null){
            res.add(MarkovClusteringBuilder.MarkovParameters.MARKOV_GAMMA_EXP);
            res.add(gamma_exp);
        }
        if(loop_gain != null){
            res.add(MarkovClusteringBuilder.MarkovParameters.MARKOV_LOOP_GAIN);
            res.add(loop_gain);
        }
        if(zero_max != null){
            res.add(MarkovClusteringBuilder.MarkovParameters.MARKOV_ZERO_MAX);
            res.add(zero_max);
        }
        return res.toArray();
    }

    @Override
    public boolean isValid() {
        return true;
    }

    public Optional<Double> getMaxResidual() {
        return Optional.ofNullable(max_residual);
    }

    public Optional<Double> getGammaExp() {
        return Optional.ofNullable(gamma_exp);
    }

    public Optional<Double> getLoopGain() {
        return Optional.ofNullable(loop_gain);
    }

    public Optional<Double> getZeroMax() {
        return Optional.ofNullable(zero_max);
    }

    public void setMaxResidual(Double max_residual) {
        this.max_residual = max_residual;
    }

    public void setGammaExp(Double gamma_exp) {
        this.gamma_exp = gamma_exp;
    }

    public void setLoopGain(Double loop_gain) {
        this.loop_gain = loop_gain;
    }

    public void setZeroMax(Double zero_max) {
        this.zero_max = zero_max;
    }

    @Override
    public String toString() {
        return "Markov{" +
                "max_residual=" + max_residual +
                ", gamma_exp=" + gamma_exp +
                ", loop_gain=" + loop_gain +
                ", zero_max=" + zero_max +
                '}';
    }
}
