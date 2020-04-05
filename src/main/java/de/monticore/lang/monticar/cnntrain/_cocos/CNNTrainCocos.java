/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTCNNTrainNode;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.se_rwth.commons.logging.Log;

public class CNNTrainCocos {

    public static CNNTrainCoCoChecker createChecker() {
        return new CNNTrainCoCoChecker()
                .addCoCo(new CheckEntryRepetition())
                .addCoCo(new CheckInteger())
                .addCoCo(new CheckRLParameterOnlyWithLearningMethodSet())
                .addCoCo(new CheckFixTargetNetworkRequiresInterval())
                .addCoCo(new CheckReinforcementRequiresEnvironment())
                .addCoCo(new CheckLearningParameterCombination())
                .addCoCo(new CheckRosEnvironmentRequiresRewardFunction())
                .addCoCo(new CheckActorCriticRequiresCriticNetwork())
                .addCoCo(new CheckRlAlgorithmParameter())
                .addCoCo(new CheckDiscreteRLAlgorithmUsesDiscreteStrategy())
                .addCoCo(new CheckContinuousRLAlgorithmUsesContinuousStrategy())
                .addCoCo(new CheckRosEnvironmentHasOnlyOneRewardSpecification())
                .addCoCo(new CheckConstraintDistributionQNetworkDependency())
                .addCoCo(new CheckConstraintLossesQNetworkDependency())
                .addCoCo(new CheckGeneratorLossTargetNameDependency())
                .addCoCo(new CheckNoiseInputDistributionDependency())
                .addCoCo(new CheckNoiseInputMissing());
    }

    public static void checkAll(CNNTrainCompilationUnitSymbol compilationUnit){
        ASTCNNTrainNode node = (ASTCNNTrainNode) compilationUnit.getAstNode().get();
        int findings = Log.getFindings().size();
        createChecker().checkAll(node);
    }

    public static void checkTrainedArchitectureCoCos(final ConfigurationSymbol configurationSymbol) {
        CNNTrainConfigurationSymbolChecker checker = new CNNTrainConfigurationSymbolChecker()
                .addCoCo(new CheckTrainedRlNetworkHasExactlyOneInput())
                .addCoCo(new CheckTrainedRlNetworkHasExactlyOneOutput())
                .addCoCo(new CheckOUParameterDimensionEqualsActionDimension())
                .addCoCo(new CheckTrainedArchitectureHasVectorAction());
        checker.checkAll(configurationSymbol);
    }

    public static void checkCriticCocos(final ConfigurationSymbol configurationSymbol) {
        CNNTrainConfigurationSymbolChecker checker = new CNNTrainConfigurationSymbolChecker()
                .addCoCo(new CheckCriticNetworkHasExactlyAOneDimensionalOutput())
                .addCoCo(new CheckCriticNetworkInputs());
        checker.checkAll(configurationSymbol);
    }

    public static void checkGANCocos(final ConfigurationSymbol configurationSymbol) {
        CNNTrainConfigurationSymbolChecker checker = new CNNTrainConfigurationSymbolChecker()
                .addCoCo(new CheckGANDiscriminatorQNetworkDependency())
                .addCoCo(new CheckGANGeneratorDiscriminatorDependency())
                .addCoCo(new CheckGANGeneratorHasOneOutput())
                .addCoCo(new CheckGANGeneratorQNetworkDependency())
                .addCoCo(new CheckGANQNetworkhasOneInput());
        checker.checkAll(configurationSymbol);
    }
}