/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator.util;

import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchSymbolCompiler;
import de.monticore.lang.monticar.cnnarch.gluongenerator.CNNArch2GluonArchitectureSupportChecker;
import de.monticore.lang.monticar.cnnarch.gluongenerator.CNNArch2GluonLayerSupportChecker;
import de.monticore.lang.monticar.cnnarch.gluongenerator.annotations.ArchitectureAdapter;
import de.monticore.lang.monticar.cnntrain._symboltable.NNArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain.annotations.Range;

import java.nio.file.Path;
import java.util.List;
import java.util.Map;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class NNArchitectureMockFactory {

    public static NNArchitectureSymbol createNNArchitectureMock() {
        NNArchitectureSymbol trainedArchitecture = mock(NNArchitectureSymbol.class);

        final String inputName = "state";
        final String outputName = "action";

        Map<String, List<Integer>> dimensionMap = ImmutableMap.<String, List<Integer>> builder()
            .put(inputName, Lists.newArrayList(8))
            .put(outputName, Lists.newArrayList(3))
            .build();

        Range stateRange = Range.withInfinityLimits();
        Range actionRange = Range.withLimits(-1, 1);
        Map<String, Range> rangeMap = ImmutableMap.<String, Range>builder()
            .put(inputName, stateRange)
            .put(outputName, actionRange)
            .build();

        Map<String, String> typeMap = ImmutableMap.<String, String>builder()
            .put(inputName, "Q")
            .put(outputName, "Q")
            .build();

        when(trainedArchitecture.getInputs()).thenReturn(Lists.newArrayList(inputName));
        when(trainedArchitecture.getOutputs()).thenReturn(Lists.newArrayList(outputName));
        when(trainedArchitecture.getDimensions()).thenReturn(dimensionMap);
        when(trainedArchitecture.getRanges()).thenReturn(rangeMap);
        when(trainedArchitecture.getTypes()).thenReturn(typeMap);

        return trainedArchitecture;
    }

    public static NNArchitectureSymbol createArchitectureSymbolByCNNArchModel(final Path modelsDirPath,
                                                                       final String rootModel) {
        CNNArchSymbolCompiler symbolCompiler = new CNNArchSymbolCompiler(new CNNArch2GluonArchitectureSupportChecker(),
                new CNNArch2GluonLayerSupportChecker());
        ArchitectureSymbol architectureSymbol = symbolCompiler.compileArchitectureSymbolFromModelsDir(modelsDirPath, rootModel);
        architectureSymbol.setComponentName(rootModel);
        return new ArchitectureAdapter(rootModel, architectureSymbol);
    }
}
