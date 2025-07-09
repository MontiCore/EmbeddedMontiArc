/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.ast.ASTCNode;
import de.monticore.lang.monticar.cnntrain._ast.*;
import de.monticore.lang.monticar.cnntrain._parser.CNNTrainAntlrParser;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;
import org.antlr.v4.runtime.misc.Pair;

import java.util.*;
import java.util.stream.Collectors;

public class CNNTrainSymbolTableCreator extends CNNTrainSymbolTableCreatorTOP {

    private String compilationUnitPackage = "";
    private ConfigurationSymbol configuration;


    public CNNTrainSymbolTableCreator(final ResolvingConfiguration resolvingConfig,
                                      final MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public CNNTrainSymbolTableCreator(final ResolvingConfiguration resolvingConfig,
                                      final Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }


    @Override
    public void visit(final ASTCNNTrainCompilationUnit compilationUnit) {
        Log.debug("Building Symboltable for Script: " + compilationUnit.getName(),
                CNNTrainSymbolTableCreator.class.getSimpleName());

        List<ImportStatement> imports = new ArrayList<>();

        this.compilationUnitPackage = Names.getQualifiedName(compilationUnit.getPackageList());

        ArtifactScope artifactScope = new ArtifactScope(
                Optional.empty(),
                compilationUnitPackage,
                imports);

        putOnStack(artifactScope);

        CNNTrainCompilationUnitSymbol compilationUnitSymbol = new CNNTrainCompilationUnitSymbol(compilationUnit.getName());
        addToScopeAndLinkWithNode(compilationUnitSymbol, compilationUnit);
    }


    @Override
    public void endVisit(ASTCNNTrainCompilationUnit ast) {
        CNNTrainCompilationUnitSymbol compilationUnitSymbol = (CNNTrainCompilationUnitSymbol) ast.getSymbolOpt().get();
        compilationUnitSymbol.setConfiguration((ConfigurationSymbol) ast.getConfiguration().getSymbolOpt().get());
        setEnclosingScopeOfNodes(ast);
    }

    @Override
    public void visit(final ASTConfiguration node){
        configuration = new ConfigurationSymbol();
        addToScopeAndLinkWithNode(configuration , node);
    }

    @Override
    public void endVisit(final ASTConfiguration trainingConfiguration) {
        removeCurrentScope();
    }
    
    @Override
    public void visit(ASTInitializerEntry node) {
        processMultiParamConfigVisit(node, node.getValue().getName());
    }

    @Override
    public void endVisit(ASTInitializerEntry node) {
        processMultiParamConfigEndVisit(node);
    }

    @Override
    public void visit(ASTCriticInitializerEntry node) {
        processMultiParamConfigVisit(node, node.getValue().getName());
    }

    @Override
    public void endVisit(ASTCriticInitializerEntry node) {
        processMultiParamConfigEndVisit(node);
    }

    @Override
    public void visit(ASTOptimizerEntry node) {
        OptimizerSymbol optimizer = new OptimizerSymbol(node.getValue().getName());
        configuration.setOptimizer(optimizer);
        addToScopeAndLinkWithNode(optimizer, node);
    }

    @Override
    public void endVisit(ASTOptimizerEntry node) {
        for (ASTEntry nodeParam : node.getValue().getParamsList()) {
            OptimizerParamSymbol param = new OptimizerParamSymbol();
            OptimizerParamValueSymbol valueSymbol = (OptimizerParamValueSymbol) nodeParam.getValue().getSymbolOpt().get();
            param.setValue(valueSymbol);
            configuration.getOptimizer().getOptimizerParamMap().put(nodeParam.getName(), param);
        }

    }

    @Override
    public void visit(ASTCriticOptimizerEntry node) {
        OptimizerSymbol optimizerSymbol = new OptimizerSymbol(node.getValue().getName());
        configuration.setCriticOptimizer(optimizerSymbol);
        addToScopeAndLinkWithNode(optimizerSymbol, node);
    }

    @Override
    public void endVisit(ASTCriticOptimizerEntry node) {
        assert configuration.getCriticOptimizer().isPresent(): "Critic optimizer not present";
        for (ASTEntry paramNode : node.getValue().getParamsList()) {
            OptimizerParamSymbol param = new OptimizerParamSymbol();
            OptimizerParamValueSymbol valueSymbol = (OptimizerParamValueSymbol)paramNode.getValue().getSymbolOpt().get();
            param.setValue(valueSymbol);
            configuration.getCriticOptimizer().get().getOptimizerParamMap().put(paramNode.getName(), param);
        }
    }

    @Override
    public void visit(ASTDiscriminatorOptimizerEntry node) {
        OptimizerSymbol optimizerSymbol = new OptimizerSymbol(node.getValue().getName());
        configuration.setCriticOptimizer(optimizerSymbol);
        addToScopeAndLinkWithNode(optimizerSymbol, node);
    }

    @Override
    public void endVisit(ASTDiscriminatorOptimizerEntry node) {
        assert configuration.getCriticOptimizer().isPresent(): "Critic optimizer not present";
        for (ASTEntry paramNode : node.getValue().getParamsList()) {
            OptimizerParamSymbol param = new OptimizerParamSymbol();
            OptimizerParamValueSymbol valueSymbol = (OptimizerParamValueSymbol)paramNode.getValue().getSymbolOpt().get();
            param.setValue(valueSymbol);
            configuration.getCriticOptimizer().get().getOptimizerParamMap().put(paramNode.getName(), param);
        }
    }

    @Override
    public void endVisit(ASTNumEpochEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTKValueEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTNoiseInputEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForString(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTGeneratorLossWeightEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForDouble(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTDiscriminatorLossWeightEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForDouble(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTPrintImagesEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForBoolean(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTGeneratorTargetNameEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForString(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTBatchSizeEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTLoadCheckpointEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForBoolean(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTCheckpointPeriodEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTLoadPretrainedEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForBoolean(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTLogPeriodEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTNormalizeEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForBoolean(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTShuffleDataEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForBoolean(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTClipGlobalGradNormEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForDouble(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTTrainContextEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        ValueSymbol value = new ValueSymbol();
        if (node.getValue().isPresentCpu()){
            value.setValue(Context.CPU);
        }
        else {
            value.setValue(Context.GPU);
        }
        entry.setValue(value);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTEvalMetricEntry node) {
        processMultiParamConfigVisit(node, node.getValue().getName());
    }

    @Override
    public void endVisit(ASTEvalMetricEntry node) {
        processMultiParamConfigEndVisit(node);
    }

    @Override
    public void endVisit(ASTEvalTrainEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForBoolean(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTLossEntry node) {
        LossSymbol loss = new LossSymbol(node.getValue().getName());
        configuration.setLoss(loss);
        addToScopeAndLinkWithNode(loss, node);
    }

    @Override
    public void endVisit(ASTLossEntry node) {
        for (ASTEntry nodeParam : node.getValue().getParamsList()) {
            LossParamSymbol param = new LossParamSymbol();
            OptimizerParamValueSymbol valueSymbol = (OptimizerParamValueSymbol) nodeParam.getValue().getSymbolOpt().get();
            LossParamValueSymbol lossParamValue = new LossParamValueSymbol();
            lossParamValue.setValue(valueSymbol.getValue());
            param.setValue(lossParamValue);
            configuration.getLoss().getLossParamMap().put(nodeParam.getName(), param);
        }
    }

	@Override
    public void endVisit(ASTLossWeightsEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForDoubleVector(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTLRPolicyValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
        if (node.isPresentFixed()){
            value.setValue(LRPolicy.FIXED);
        }
        else if (node.isPresentExp()){
            value.setValue(LRPolicy.EXP);
        }
        else if (node.isPresentInv()){
            value.setValue(LRPolicy.INV);
        }
        else if (node.isPresentStep()){
            value.setValue(LRPolicy.STEP);
        }
        else if (node.isPresentSigmoid()){
            value.setValue(LRPolicy.SIGMOID);
        }
        else if (node.isPresentPoly()){
            value.setValue(LRPolicy.POLY);
        }
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTNumberValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
        Double number = node.getNumberWithUnit().getNumber().get();
        value.setValue(number);
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTIntegerValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
        Integer number = getIntegerFromNumber(node);
        value.setValue(number);
        addToScopeAndLinkWithNode(value, node);
    }

    @Override
    public void endVisit(ASTBooleanValue node) {
        OptimizerParamValueSymbol value = new OptimizerParamValueSymbol();
        if (node.isPresentTRUE()){
            value.setValue(true);
        }
        else if (node.isPresentFALSE()){
            value.setValue(false);
        }
        addToScopeAndLinkWithNode(value, node);
    }


    private ValueSymbol getValueSymbolForInteger(ASTIntegerValue astIntegerValue) {
        ValueSymbol value = new ValueSymbol();
        Integer value_as_int = getIntegerFromNumber(astIntegerValue);
        value.setValue(value_as_int);
        return value;
    }

    private ValueSymbol getValueSymbolForDouble(final ASTNumberValue astNumberValue) {
        ValueSymbol valueSymbol = new ValueSymbol();
        Double value = getDoubleFromNumber(astNumberValue);
        valueSymbol.setValue(value);
        return valueSymbol;
    }

    private ValueSymbol getValueSymbolForBoolean(ASTBooleanValue astBooleanValue) {
        ValueSymbol value = new ValueSymbol();
        Boolean value_as_bool = getBooleanForValue(astBooleanValue);
        value.setValue(value_as_bool);
        return value;
    }

    private ValueSymbol getValueSymbolForString(ASTStringValue astStringValue) {
        ValueSymbol value = new ValueSymbol();
        String valueAsString = getStringFromStringValue(astStringValue);
        value.setValue(valueAsString);
        return value;
    }

    private ValueSymbol getValueSymbolForDoubleVector(ASTDoubleVectorValue astDoubleVectorValue) {
        ValueSymbol value = new ValueSymbol();
		List<Double> value_as_double_list = getDoubleVectorFromList(astDoubleVectorValue);
        value.setValue(value_as_double_list);
        return value;
    }

    private ValueSymbol getValueSymbolForComponentName(ASTComponentNameValue astComponentNameValue) {
        ValueSymbol value = new ValueSymbol();
        List<String> valueAsList = astComponentNameValue.getNameList();
        value.setValue(valueAsList);
        return value;
    }

    private ValueSymbol getValueSymbolForComponentNameAsString(ASTComponentNameValue astComponentNameValue) {
        ValueSymbol value = new ValueSymbol();
        value.setValue(String.join(".", astComponentNameValue.getNameList()));
        return value;
    }

    private String getStringFromStringValue(ASTStringValue value) {
        return value.getStringLiteral().getValue();
    }

    private int getIntegerFromNumber(ASTIntegerValue value) {
        return value.getNumberWithUnit().getNumber().get().intValue();
    }

    private double getDoubleFromNumber(final ASTNumberValue value) {
        assert value.getNumberWithUnit().getNumber().isPresent() : "Number value is not present";
        return value.getNumberWithUnit().getNumber().get();
    }

    private boolean getBooleanForValue(ASTBooleanValue value2) {
        return value2.isPresentTRUE();
    }

    private List<Double> getDoubleVectorFromList(ASTDoubleVectorValue value) {
        return value.getNumberList().stream()
                .filter(n -> n.getNumber().isPresent())
                .map(n -> n.getNumber().get())
                .collect(Collectors.toList());
    }

    private List<Integer> getIntegerListFromValue(ASTIntegerListValue value) {
        return value.getNumberList().stream()
                .filter(n -> n.getNumber().isPresent())
                .map(n -> n.getNumber().get().intValue())
                .collect(Collectors.toList());
    }

    @Override
    public void endVisit(ASTUseTeacherForcing node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForBoolean(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void endVisit(ASTSaveAttentionImage node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForBoolean(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTLearningMethodEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        ValueSymbol value = new ValueSymbol();

        if (node.getValue().isPresentReinforcement()) {
            value.setValue(LearningMethod.REINFORCEMENT);
        } if (node.getValue().isPresentGan()) {
            value.setValue(LearningMethod.GAN);
        } else if (node.getValue().isPresentSupervisedLearning()) {
            value.setValue(LearningMethod.SUPERVISED);
        }

        entry.setValue(value);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTGeneratorLossEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        ValueSymbol value = new ValueSymbol();

        if (node.getValue().isPresentL1()) {
            value.setValue(GeneratorLoss.L1);
        } else if (node.getValue().isPresentL2()) {
            value.setValue(GeneratorLoss.L2);
        }

        entry.setValue(value);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTRLAlgorithmEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        ValueSymbol value = new ValueSymbol();

        if (node.getValue().isPresentDdpg()) {
            value.setValue(RLAlgorithm.DDPG);
        } else if(node.getValue().isPresentTdThree()) {
            value.setValue(RLAlgorithm.TD3);
        } else {
            value.setValue(RLAlgorithm.DQN);
        }

        entry.setValue(value);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTNumEpisodesEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTDiscountFactorEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForDouble(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTNumMaxStepsEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTTargetScoreEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForDouble(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTTrainingIntervalEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTUseFixTargetNetworkEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForBoolean(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTTargetNetworkUpdateIntervalEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTSnapshotIntervalEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTAgentNameEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForString(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTUseDoubleDQNEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForBoolean(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTCriticNetworkEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForComponentNameAsString(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTDiscriminatorNetworkEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForComponentNameAsString(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTQNetworkEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForComponentNameAsString(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }


    @Override
    public void visit(ASTReplayMemoryEntry node) {
        processMultiParamConfigVisit(node, node.getValue().getName());
    }

    @Override
    public void endVisit(ASTReplayMemoryEntry node) {
        processMultiParamConfigEndVisit(node);
    }

    @Override
    public void visit(ASTStrategyEntry node) {
        processMultiParamConfigVisit(node, node.getValue().getName());
    }

    @Override
    public void endVisit(ASTStrategyEntry node) {
        processMultiParamConfigEndVisit(node);
    }

    @Override
    public void visit(ASTEnvironmentEntry node) {
        Environment environment;
        environment = node.getValue().getName().equals("ros_interface") ? Environment.ROS_INTERFACE : Environment.GYM;
        processMultiParamConfigVisit(node, environment);
    }

    @Override
    public void endVisit(ASTEnvironmentEntry node) {
        processMultiParamConfigEndVisit(node);
    }

    @Override
    public void visit(ASTConstraintDistributionEntry node) {
        processMultiParamMapConfigVisit(node, node.getName());
    }

    @Override
    public void endVisit(ASTConstraintDistributionEntry node) {
        processMultiParamMapConfigEndVisit(node);
    }

    @Override
    public void visit(ASTConstraintLossEntry node) {
        processMultiParamMapConfigVisit(node, node.getName());
    }

    @Override
    public void endVisit(ASTConstraintLossEntry node) {
        processMultiParamMapConfigEndVisit(node);
    }

    @Override
    public void visit(ASTNoiseDistributionEntry node) {
        NoiseDistribution noiseDistribution;
        if(node.getValue().getName().equals("gaussian"))
            noiseDistribution = NoiseDistribution.GAUSSIAN;
        else if (node.getValue().getName().equals("uniform"))
            noiseDistribution = NoiseDistribution.UNIFORM;
        else
            noiseDistribution = NoiseDistribution.GAUSSIAN;

        processMultiParamConfigVisit(node, noiseDistribution);
    }

    @Override
    public void endVisit(ASTNoiseDistributionEntry node) {
        processMultiParamConfigEndVisit(node);
    }


    @Override
    public void visit(ASTRewardFunctionEntry node) {
        RewardFunctionSymbol symbol = new RewardFunctionSymbol(node.getName());
        symbol.setRewardFunctionComponentName(node.getValue().getNameList());
        configuration.setRlRewardFunction(symbol);
        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void visit(ASTPreprocessingEntry node) {
        PreprocessingComponentSymbol symbol = new PreprocessingComponentSymbol(node.getName());
        symbol.setPreprocessingComponentName(node.getValue().getNameList());
        configuration.setPreprocessingComponent(symbol);
        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void visit(ASTSoftTargetUpdateRateEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForDouble(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTStartTrainingAtEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTEvaluationSamplesEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTPolicyNoiseEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForDouble(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTNoiseClipEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForDouble(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTStrategyGaussianNoiseVarianceEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForDouble(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    @Override
    public void visit(ASTPolicyDelayEntry node) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        entry.setValue(getValueSymbolForInteger(node.getValue()));
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    private void processMultiParamConfigVisit(ASTMultiParamConfigEntry node, Object value) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        MultiParamValueSymbol valueSymbol = new MultiParamValueSymbol();
        valueSymbol.setValue(value);
        entry.setValue(valueSymbol);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    private void processMultiParamConfigEndVisit(ASTMultiParamConfigEntry node) {
        ValueSymbol valueSymbol = configuration.getEntryMap().get(node.getName()).getValue();
        assert valueSymbol instanceof MultiParamValueSymbol : "Value symbol is not a multi parameter symbol";
        MultiParamValueSymbol multiParamValueSymbol = (MultiParamValueSymbol)valueSymbol;
        for (ASTEntry nodeParam : ((ASTMultiParamValue)node.getValue()).getParamsList()) {
            multiParamValueSymbol.addParameter(nodeParam.getName(),
                    retrievePrimitiveValueByConfigValue(nodeParam.getValue()));
        }
    }

    private void processMultiParamMapConfigVisit(ASTMultiParamValueMapConfigEntry node, Object value) {
        EntrySymbol entry = new EntrySymbol(node.getName());
        MultiParamValueMapSymbol valueSymbol = new MultiParamValueMapSymbol();
        valueSymbol.setValue(value);
        entry.setValue(valueSymbol);
        addToScopeAndLinkWithNode(entry, node);
        configuration.getEntryMap().put(node.getName(), entry);
    }

    private void processMultiParamMapConfigEndVisit(ASTMultiParamValueMapConfigEntry node) {
        ValueSymbol valueSymbol = configuration.getEntryMap().get(node.getName()).getValue();
        assert valueSymbol instanceof MultiParamValueMapSymbol : "Value symbol is not a multi parameter symbol";
        MultiParamValueMapSymbol multiParamValueMapSymbol = (MultiParamValueMapSymbol)valueSymbol;
        for (ASTConfigValue nodeParam : ((ASTMultiParamValueMapParamValue)node.getValue()).getParamsList()) {
            ASTMultiParamValueMapTupleValue tuple = ((ASTMultiParamValueMapTupleValue)nodeParam);
            ASTStringValue name = tuple.getName();
            ASTMultiParamValue multiValue = tuple.getMultiParamValue();
            String valueName = multiValue.getName();
            multiParamValueMapSymbol.addMultiParamValueName(getStringFromStringValue(name), valueName);
            HashMap<String, Object> mapEntry = new HashMap<>();
            for (ASTEntry param : multiValue.getParamsList()) {
                String valueEntryName = param.getName();
                Object res = retrievePrimitiveValueByConfigValue(param.getValue());
                mapEntry.put(valueEntryName, res);
            }
            multiParamValueMapSymbol.addParameter(getStringFromStringValue(name), mapEntry);
        }
    }

    private Object retrievePrimitiveValueByConfigValue(final ASTConfigValue configValue) {
        if (configValue instanceof ASTIntegerValue) {
            return getIntegerFromNumber((ASTIntegerValue)configValue);
        } else if (configValue instanceof ASTNumberValue) {
            return getDoubleFromNumber((ASTNumberValue)configValue);
        } else if (configValue instanceof ASTBooleanValue) {
            return getBooleanForValue((ASTBooleanValue)configValue);
        } else if (configValue instanceof ASTStringValue) {
            return getStringFromStringValue((ASTStringValue)configValue);
        } else if (configValue instanceof ASTEpsilonDecayMethodValue) {
            ASTEpsilonDecayMethodValue epsilonDecayMethodValue = (ASTEpsilonDecayMethodValue)configValue;
            if (epsilonDecayMethodValue.isPresentLinear()) {
                return EpsilonDecayMethod.LINEAR;
            } else {
                return EpsilonDecayMethod.NO;
            }
        } else if (configValue instanceof ASTDoubleVectorValue) {
            ASTDoubleVectorValue astDoubleVectorValue = (ASTDoubleVectorValue)configValue;
            return astDoubleVectorValue.getNumberList().stream()
                .filter(n -> n.getNumber().isPresent())
                .map(n -> n.getNumber().get())
                .collect(Collectors.toList());
        } else if (configValue instanceof ASTIntegerListValue) {
            return getIntegerListFromValue((ASTIntegerListValue)configValue);
        }
        throw new UnsupportedOperationException("Unknown Value type: " + configValue.getClass());
    }
}
