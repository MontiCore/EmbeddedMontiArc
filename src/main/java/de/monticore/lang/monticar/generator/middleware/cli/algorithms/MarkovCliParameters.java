package de.monticore.lang.monticar.generator.middleware.cli.algorithms;

import de.monticore.lang.monticar.generator.middleware.clustering.ClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.MarkovClusteringAlgorithm;
import de.monticore.lang.monticar.generator.middleware.clustering.algorithms.MarkovClusteringBuilder;

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

    @Override
    public String getName() {
        return TYPE_MARKOV;
    }

    @Override
    public ClusteringAlgorithm asClustringAlgorithm() {
        return new MarkovClusteringAlgorithm();
    }

    @Override
    public List<Object> asAlgorithmArgs() {
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
        return res;
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
}
