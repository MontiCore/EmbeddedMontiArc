package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.StreamScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ComponentStreamTestViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.StreamViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.TestsMainEntryViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.BooleanOutputPortCheck;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.ComponentCheckViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.IOutputPortCheck;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.RangeOutputPortCheck;
import de.monticore.lang.monticar.literals2._ast.ASTBooleanLiteral;
import de.monticore.lang.monticar.streamunits._ast.ASTNamedStreamUnits;
import de.monticore.lang.monticar.streamunits._ast.ASTPrecisionNumber;
import de.monticore.lang.monticar.streamunits._ast.ASTStream;
import de.monticore.lang.monticar.streamunits._ast.ASTStreamInstruction;
import de.monticore.lang.monticar.streamunits._ast.ASTStreamValue;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.monticar.streamunits._symboltable.NamedStreamUnitsSymbol;
import de.monticore.lang.monticar.streamunits._visitor.StreamUnitsVisitor;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import siunit.monticoresiunit.si._ast.ASTUnitNumber;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.Set;

public final class TestsGeneratorCPP {

    public static final String TESTS_DIRECTORY_NAME = "test";

    private final GeneratorCPP generator;

    TestsGeneratorCPP(GeneratorCPP generator) {
        this.generator = Log.errorIfNull(generator);
    }

    public List<FileContent> generateStreamTests(Scope symTab) {
        Set<String> testedComponents = new HashSet<>();
        List<BluePrintCPP> bluePrints = generator.getBluePrints();
        if (bluePrints == null || bluePrints.isEmpty()) {
            Log.warn("no blue prints were generated");
            return Collections.emptyList();
        }
        StreamScanner scanner = new StreamScanner(generator.getModelsDirPath(), symTab);
        Map<ComponentSymbol, Set<ComponentStreamUnitsSymbol>> availableStreams = scanner.scan();
        List<FileContent> files = new ArrayList<>();
        TestsMainEntryViewModel viewModelForMain = new TestsMainEntryViewModel();
        viewModelForMain.setIncludes(new ArrayList<>());
        for (BluePrintCPP b : bluePrints) {
            ExpandedComponentInstanceSymbol s = b.getOriginalSymbol();
            if (s != null) {
                ComponentSymbol cs = s.getComponentType().getReferencedSymbol();
                if (testedComponents.add(cs.getFullName())) {
                    Set<ComponentStreamUnitsSymbol> streamsForComponent = availableStreams.get(cs);
                    if (streamsForComponent != null && !streamsForComponent.isEmpty()) {
                        ComponentStreamTestViewModel viewModel = getStreamViewModel(b, cs, streamsForComponent);
                        String genTestCode = AllTemplates.generateComponentStreamTest(viewModel);
                        files.add(new FileContent(genTestCode, getFileName(viewModel)));
                        viewModelForMain.getIncludes().add(viewModel.getFileNameWithExtension());
                    }
                }
            } else {
                Log.warn("no symbol info for blue print " + b.getName() + " (package: " + b.getPackageName() + ")");
            }
        }
        files.add(new FileContent(AllTemplates.generateMainEntry(viewModelForMain), TESTS_DIRECTORY_NAME + "/tests_main.cpp"));
        files.add(getCatchLib());
        return files;
    }

    private static ComponentStreamTestViewModel getStreamViewModel(BluePrintCPP b, ComponentSymbol cs, Set<ComponentStreamUnitsSymbol> streamsForComponent) {
        ComponentStreamTestViewModel viewModel = new ComponentStreamTestViewModel();
        viewModel.setComponentName(b.getName());
        viewModel.setFileNameWithoutExtension(b.getName() + "_test");
        viewModel.setStreams(new ArrayList<>());
        for (ComponentStreamUnitsSymbol stream : streamsForComponent) {
            StreamViewModel svm = new StreamViewModel();
            viewModel.getStreams().add(svm);
            svm.setName(stream.getFullName());
            svm.setChecks(getComponentPortChecks(cs, stream));
        }
        return viewModel;
    }

    private static List<ComponentCheckViewModel> getComponentPortChecks(ComponentSymbol cs, ComponentStreamUnitsSymbol stream) {
        List<ComponentCheckViewModel> result = new ArrayList<>();
        Map<PortSymbol, ASTStream> port2NamedStream = new HashMap<>();
        for (PortSymbol port : cs.getPorts()) {
            NamedStreamUnitsSymbol namedStreamForPort = stream.getNamedStream(port.getName()).orElse(null);
            if (namedStreamForPort != null && namedStreamForPort.getAstNode().isPresent()) {
                ASTNamedStreamUnits node = (ASTNamedStreamUnits) namedStreamForPort.getAstNode().get();
                port2NamedStream.put(port, node.getStream());
            }
        }
        int streamLength = -1;
        for (ASTStream ns : port2NamedStream.values()) {
            int l = ns.getStreamInstructions().size();
            if (streamLength == -1) {
                streamLength = l;
            } else if (streamLength != l) {
                String msg = String.format("streams have different lengths: %s and %s (stream %s)", streamLength, l, stream.getFullName());
                Log.error(msg);
                throw new RuntimeException(msg);
            }
        }
        if (streamLength <= 0) {
            String msg = String.format("invalid stream data in %s", stream.getFullName());
            Log.error(msg);
            throw new RuntimeException(msg);
        }
        for (int i = 0; i < streamLength; i++) {
            ComponentCheckViewModel vm = new ComponentCheckViewModel();
            vm.setInputPortName2Value(new HashMap<>());
            vm.setOutputPortName2Check(new HashMap<>());
            for (Map.Entry<PortSymbol, ASTStream> kv : port2NamedStream.entrySet()) {
                ASTStreamInstruction nextInstruction = kv.getValue().getStreamInstructions().get(i);
                if (nextInstruction.getStreamValue().isPresent()) {
                    ASTStreamValue sv = nextInstruction.getStreamValue().get();
                    PortSymbol port = kv.getKey();
                    if (port.isIncoming()) {
                        ASTStreamValue2InputPortValue converter = new ASTStreamValue2InputPortValue();
                        sv.accept(converter);
                        if (converter.getResult() != null) {
                            vm.getInputPortName2Value().put(port.getName(), converter.getResult());
                        }
                    } else {
                        ASTStreamValue2OutputPortCheck converter = new ASTStreamValue2OutputPortCheck();
                        sv.accept(converter);
                        if (converter.getResult() != null) {
                            vm.getOutputPortName2Check().put(port.getName(), converter.getResult());
                        }
                    }
                }
            }
            result.add(vm);
        }
        return result;
    }

    private static FileContent getCatchLib() {
        InputStream resource = TestsGeneratorCPP.class.getResourceAsStream("/vendor/catch.hpp");
        String body = new Scanner(resource, "UTF-8").useDelimiter("\\A").next();
        return new FileContent(body, TESTS_DIRECTORY_NAME + "/catch.hpp");
    }

    private static String getFileName(ComponentStreamTestViewModel viewModel) {
        return TESTS_DIRECTORY_NAME + "/" + viewModel.getFileNameWithExtension();
    }

    private static final class ASTStreamValue2OutputPortCheck implements StreamUnitsVisitor {

        private IOutputPortCheck result = null;

        public IOutputPortCheck getResult() {
            return result;
        }

        @Override
        public void visit(ASTBooleanLiteral node) {
            if (node.getValue()) {
                result = BooleanOutputPortCheck.TRUE_EXPECTED;
            } else {
                result = BooleanOutputPortCheck.FALSE_EXPECTED;
            }
        }

        @Override
        public void visit(ASTPrecisionNumber node) {
            ASTUnitNumber unitNumber = node.getUnitNumber();
            if (!unitNumber.getNumber().isPresent()) {
                return;
            }
            double baseValue = unitNumber.getNumber().get().doubleValue();
            if (node.getPrecision().isPresent()
                    && node.getPrecision().get().getUnitNumber().getNumber().isPresent()) {
                double delta = node.getPrecision().get().getUnitNumber().getNumber().get().doubleValue();
                result = RangeOutputPortCheck.from(baseValue - delta, baseValue + delta);
            } else {
                result = RangeOutputPortCheck.from(baseValue, baseValue);
            }
        }
    }

    private static final class ASTStreamValue2InputPortValue implements StreamUnitsVisitor {

        private String result = null;

        public String getResult() {
            return result;
        }

        @Override
        public void visit(ASTBooleanLiteral node) {
            result = node.getValue() ? "true" : "false";
        }

        @Override
        public void visit(ASTPrecisionNumber node) {
            ASTUnitNumber unitNumber = node.getUnitNumber();
            if (!unitNumber.getNumber().isPresent()) {
                return;
            }
            result = Double.toString(unitNumber.getNumber().get().doubleValue());
        }
    }
}
