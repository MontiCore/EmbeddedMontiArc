package de.monticore.mlpipelines.automl.trainalgorithms.efficientnet;

import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.math._symboltable.expression.MathNumberExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.NetworkInstructionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.SerialCompositeElementSymbol;
import de.monticore.lang.monticar.types2._ast.ASTDimension;
import de.monticore.mlpipelines.automl.configuration.EfficientNetConfig;
import de.monticore.mlpipelines.automl.helper.ArchitectureHelper;
import de.monticore.mlpipelines.automl.helper.MathNumberExpressionWrapper;
import de.monticore.mlpipelines.automl.helper.OriginalLayerParams;
import de.monticore.mlpipelines.pipelines.Pipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ScalingFactorsGridSearch {
    private final int standardPhi = 1;

    private final NetworkScaler networkScaler;
    private final ArchitectureSymbol architecture;
    private final Pipeline trainPipeline;
    private final EfficientNetConfig config;
    private ScalingFactors bestScalingFactors;
    private double bestAccuracy;

    public ScalingFactorsGridSearch(
            ArchitectureSymbol architecture,
            EfficientNetConfig config,
            Pipeline networkTrainer,
            NetworkScaler networkScaler) {
        this.architecture = architecture;
        this.config = config;
        this.trainPipeline = networkTrainer;
        this.networkScaler = networkScaler;
    }

    public ScalingFactors findBestScalingFactors() {
        initVariables();
        List<ScalingFactors> possibleScalingFactors = getPossibleScalingFactors();

        for (int i = 0; i < possibleScalingFactors.size(); i++) {
            ScalingFactors scalingFactors = possibleScalingFactors.get(i);
            System.out.println();
            System.out.println("Iteration " + (i + 1) + " of " + possibleScalingFactors.size());
            System.out.println("Test these scaling factors: " + scalingFactors.toString());
            checkScalingFactors(scalingFactors);
        }

        System.out.println();
        System.out.println("Best scaling factors: " + bestScalingFactors.toString());
        System.out.println("Best scalingFactors accuracy: " + bestAccuracy);
        return bestScalingFactors;
    }

    public List<ScalingFactors> getPossibleScalingFactors() {
        List<ScalingFactors> scalingFactors = new ArrayList<>();
        for (double gamma = config.getMinScalingFactors().gamma;
             gamma <= config.getMaxScalingFactors().gamma;
             gamma += config.getScalingFactorsStepSize().gamma) {

            for (double beta = config.getMinScalingFactors().beta;
                 beta <= config.getMaxScalingFactors().beta;
                 beta += config.getScalingFactorsStepSize().beta) {

                float alpha = alphaFromFlopsCondition(beta, gamma);
                scalingFactors.add(new ScalingFactors(alpha, beta, gamma));
            }
        }

        return scalingFactors;
    }

    private float alphaFromFlopsCondition(double beta, double gamma) {
        float foundAlpha = (float) (config.getFlopsConditionValue() / Math.pow(beta * gamma, 2));
        return foundAlpha;
    }

    private void checkScalingFactors(ScalingFactors scalingFactors) {
        if (scalingFactors.alpha < config.getMinScalingFactors().alpha) {
            return;
        }

        this.networkScaler.scale(this.architecture, scalingFactors, this.standardPhi);
        trainPipeline.execute();
        this.networkScaler.rollbackScaledNetwork();
        checkIfScalingFactorsAreBetterThanBest(scalingFactors);
    }

    private void checkIfScalingFactorsAreBetterThanBest(ScalingFactors scalingFactors) {
        float newAccuracy = trainPipeline.getTrainedAccuracy();
        if (trainPipeline.getTrainedAccuracy() > bestAccuracy) {
            bestAccuracy = newAccuracy;
            bestScalingFactors = scalingFactors;
        }
    }

    private void initVariables() {
        this.bestScalingFactors = new ScalingFactors(1, 1, 1);
        this.bestAccuracy = 0;
    }
}
