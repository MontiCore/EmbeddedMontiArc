/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnntrain.cocos;

import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import de.monticore.lang.monticar.cnntrain._symboltable.NNArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain.annotations.Range;

import java.awt.*;
import java.util.List;
import java.util.Map;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class NNArchitecturerBuilder {
    private static final String ACTOR_NN_NAME ="ActorNetwork";
    private static final String ACTOR_STATE_NAME = "actorState";
    private static final List<Integer> ACTOR_STATE_DIM = Lists.newArrayList(25);
    private static final String ACTOR_STATE_TYPE = "Q";
    private static final Range ACTOR_STATE_RANGE = Range.withInfinityLimits();

    private static final String ACTOR_ACTION_NAME = "actorAction";
    private static final List<Integer> ACTOR_ACTION_DIM = Lists.newArrayList(3);
    private static final String ACTOR_ACTION_TYPE = "Q";
    private static final Range ACTOR_ACTION_RANGE = Range.withLimits(0, 1);

    private static final String CRITIC_NN_NAME = "CriticNetwork";
    private static final String CRITIC_STATE_NAME = "criticState";
    private static final String CRITIC_ACTION_NAME = "criticAction";
    private static final String CRITIC_QVALUE_NAME = "criticQValue";

    public NNArchitectureSymbol getCriticWithDifferentStateRanges() {
        Map<String, Range> ranges = getValidCriticRanges();
        ranges.put(CRITIC_STATE_NAME, Range.withLowerInfinityLimit(5.0));
        return getNNArchitectureSymbolFrom(CRITIC_NN_NAME, getValidCriticInputs(), getValidCriticOutputs(),
            getValidCriticDimensions(), getValidCriticTypes(), ranges);
    }

    public NNArchitectureSymbol getCriticWithDifferentActionRanges() {
        Map<String, Range> ranges = getValidCriticRanges();
        ranges.put(CRITIC_ACTION_NAME, Range.withLimits(-3.5, 3.5));
        return getNNArchitectureSymbolFrom(CRITIC_NN_NAME, getValidCriticInputs(), getValidCriticOutputs(),
            getValidCriticDimensions(), getValidCriticTypes(), ranges);
    }

    public NNArchitectureSymbol getCriticWithDifferentActionTypes() {
        Map<String, String> types = getValidCriticTypes();
        types.put(CRITIC_ACTION_NAME, "Z");
        return getNNArchitectureSymbolFrom(CRITIC_NN_NAME, getValidCriticInputs(), getValidCriticOutputs(),
            getValidCriticDimensions(), types, getValidCriticRanges());
    }

    public NNArchitectureSymbol getCriticWithDifferentStateTypes() {
        Map<String, String> types = getValidCriticTypes();
        types.put(CRITIC_STATE_NAME, "Z");
        return getNNArchitectureSymbolFrom(CRITIC_NN_NAME, getValidCriticInputs(), getValidCriticOutputs(),
            getValidCriticDimensions(), types, getValidCriticRanges());
    }

    public NNArchitectureSymbol getCriticWithDifferentActionDimensions() {
        Map<String, List<Integer>> dimensions = getValidCriticDimensions();
        dimensions.put(CRITIC_ACTION_NAME, Lists.newArrayList(28));
        return getNNArchitectureSymbolFrom(CRITIC_NN_NAME, getValidCriticInputs(), getValidCriticOutputs(), dimensions,
            getValidCriticTypes(), getValidCriticRanges());
    }

    public NNArchitectureSymbol getCriticWithDifferentStateDimensions() {
        Map<String, List<Integer>> dimensions = getValidCriticDimensions();
        dimensions.put(CRITIC_STATE_NAME, Lists.newArrayList(12));
        return getNNArchitectureSymbolFrom(CRITIC_NN_NAME, getValidCriticInputs(), getValidCriticOutputs(), dimensions,
            getValidCriticTypes(), getValidCriticRanges());
    }

    public NNArchitectureSymbol getCriticWithTwoOutputs() {
        final String anySecondOutputName = "qvalue2";
        List<String> outputNames = getValidCriticOutputs();
        outputNames.add(anySecondOutputName);
        Map<String, List<Integer>> dimensions = getValidCriticDimensions();
        dimensions.put(anySecondOutputName, Lists.newArrayList(2));
        Map<String, String> types = getValidCriticTypes();
        types.put(anySecondOutputName, "Q");
        Map<String, Range> ranges = getValidCriticRanges();
        ranges.put(anySecondOutputName, Range.withInfinityLimits());

        return getNNArchitectureSymbolFrom(ACTOR_NN_NAME, getValidCriticInputs(), outputNames, dimensions, types, ranges);
    }

    public NNArchitectureSymbol getCriticWithThreeDimensionalOutput() {
        Map<String, List<Integer>> dimensions = getValidCriticDimensions();
        dimensions.put(CRITIC_QVALUE_NAME, Lists.newArrayList(4));
        return getNNArchitectureSymbolFrom(CRITIC_NN_NAME, getValidCriticInputs(), getValidCriticOutputs(),
            dimensions, getValidCriticTypes(), getValidCriticRanges());
    }

    public NNArchitectureSymbol getTrainedArchitectureWithTwoOutputs() {
        final String anySecondOutputName = "action2";
        List<String> outputNames = getValidActorOutputs();
        outputNames.add(anySecondOutputName);
        Map<String, List<Integer>> dimensions = getValidActorDimensions();
        dimensions.put(anySecondOutputName, Lists.newArrayList(2));
        Map<String, String> types = getValidActorTypes();
        types.put(anySecondOutputName, "Q");
        Map<String, Range> ranges = getValidActorRanges();
        ranges.put(anySecondOutputName, Range.withInfinityLimits());

        return getNNArchitectureSymbolFrom(ACTOR_NN_NAME, getValidActorInputs(), outputNames, dimensions, types, ranges);
    }

    public NNArchitectureSymbol getTrainedArchitectureWithTwoInputs() {
        final String anySecondInputName = "state2";
        List<String> inputNames = getValidActorInputs();
        inputNames.add(anySecondInputName);
        Map<String, List<Integer>> dimensions = getValidActorDimensions();
        dimensions.put(anySecondInputName, Lists.newArrayList(2));
        Map<String, String> types = getValidActorTypes();
        types.put(anySecondInputName, "Q");
        Map<String, Range> ranges = getValidActorRanges();
        ranges.put(anySecondInputName, Range.withInfinityLimits());

        return getNNArchitectureSymbolFrom(ACTOR_NN_NAME, inputNames, getValidActorOutputs(), dimensions, types, ranges);

    }

    public NNArchitectureSymbol getNNArchitectureSymbolFrom(String name, List<String> inputs, List<String> outputs,
        Map<String, List<Integer>> dimensions, Map<String, String> types, Map<String, Range> ranges)
    {
        NNArchitectureSymbol architectureSymbolMock = mock(NNArchitectureSymbol.class);
        when(architectureSymbolMock.getName()).thenReturn(name);
        when(architectureSymbolMock.getInputs()).thenReturn(inputs);
        when(architectureSymbolMock.getOutputs()).thenReturn(outputs);
        when(architectureSymbolMock.getDimensions()).thenReturn(dimensions);
        when(architectureSymbolMock.getTypes()).thenReturn(types);
        when(architectureSymbolMock.getRanges()).thenReturn(ranges);

        return architectureSymbolMock;
    }

    public NNArchitectureSymbol getValidTrainedArchitecture() {
        return getNNArchitectureSymbolFrom(ACTOR_NN_NAME,
            getValidActorInputs(), getValidActorOutputs(), getValidActorDimensions(), getValidActorTypes(),
            getValidActorRanges());
    }

    public Map<String, Range> getValidActorRanges() {
        return Maps.newHashMap(ImmutableMap.<String, Range>builder()
            .put(ACTOR_STATE_NAME, ACTOR_STATE_RANGE)
            .put(ACTOR_ACTION_NAME, ACTOR_ACTION_RANGE)
            .build());
    }

    public Map<String, List<Integer>> getValidActorDimensions() {
        return Maps.newHashMap(ImmutableMap.<String, List<Integer>>builder()
            .put(ACTOR_STATE_NAME, ACTOR_STATE_DIM)
            .put(ACTOR_ACTION_NAME, ACTOR_ACTION_DIM)
            .build());
    }

    public List<String> getValidActorInputs() {
        return Lists.newArrayList(ACTOR_STATE_NAME);
    }

    public List<String> getValidActorOutputs() {
        return Lists.newArrayList(ACTOR_ACTION_NAME);
    }

    public Map<String, String> getValidActorTypes() {
        return Maps.newHashMap(ImmutableMap.<String, String>builder()
            .put(ACTOR_STATE_NAME, ACTOR_STATE_TYPE)
            .put(ACTOR_ACTION_NAME, ACTOR_ACTION_TYPE)
            .build());
    }

    public List<String> getValidCriticInputs() {
        return Lists.newArrayList(CRITIC_STATE_NAME, CRITIC_ACTION_NAME);
    }

    public List<String> getValidCriticOutputs() {
        return Lists.newArrayList(CRITIC_QVALUE_NAME);
    }

    public Map<String, List<Integer>> getValidCriticDimensions() {
        return Maps.newHashMap(ImmutableMap.<String, List<Integer>>builder()
            .put(CRITIC_STATE_NAME, ACTOR_STATE_DIM)
            .put(CRITIC_ACTION_NAME, ACTOR_ACTION_DIM)
            .put(CRITIC_QVALUE_NAME, Lists.newArrayList(1))
            .build());
    }

    public Map<String, String> getValidCriticTypes() {
        return Maps.newHashMap(ImmutableMap.<String, String>builder()
            .put(CRITIC_STATE_NAME, ACTOR_STATE_TYPE)
            .put(CRITIC_ACTION_NAME, ACTOR_ACTION_TYPE)
            .put(CRITIC_QVALUE_NAME, "Q")
            .build());
    }

    public Map<String, Range> getValidCriticRanges() {
        return Maps.newHashMap(ImmutableMap.<String, Range>builder()
            .put(CRITIC_STATE_NAME, ACTOR_STATE_RANGE)
            .put(CRITIC_ACTION_NAME, ACTOR_ACTION_RANGE)
            .put(CRITIC_QVALUE_NAME, Range.withInfinityLimits())
            .build());
    }

    public NNArchitectureSymbol getValidCriticArchitecture() {
        return getNNArchitectureSymbolFrom(CRITIC_NN_NAME, getValidCriticInputs(), getValidCriticOutputs(),
            getValidCriticDimensions(), getValidCriticTypes(), getValidCriticRanges());
    }

    public NNArchitectureSymbol getTrainedArchitectureWithMultidimensionalAction() {
        Map<String, List<Integer>> dimensions = getValidActorDimensions();
        dimensions.put(ACTOR_ACTION_NAME, Lists.newArrayList(2, 2, 3));
        return getNNArchitectureSymbolFrom(ACTOR_NN_NAME, getValidActorInputs(), getValidActorOutputs(),
            dimensions, getValidCriticTypes(), getValidCriticRanges());
    }

    public NNArchitectureSymbol getCriticWithMultidimensionalAction() {
        Map<String, List<Integer>> dimensions = getValidCriticDimensions();
        dimensions.put(CRITIC_ACTION_NAME, Lists.newArrayList(2, 2, 3));
        return getNNArchitectureSymbolFrom(CRITIC_NN_NAME, getValidCriticInputs(), getValidCriticOutputs(),
            dimensions, getValidCriticTypes(), getValidCriticRanges());

    }
}
