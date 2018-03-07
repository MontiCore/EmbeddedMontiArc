package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.ComponentScanner;
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
import de.monticore.lang.numberunit._ast.ASTUnitNumber;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public final class TestsGeneratorCPP {

    public static final String TESTS_DIRECTORY_NAME = "/test";

    private final GeneratorCPP generator;
    private List<BluePrintCPP> bluePrints;
    public static Map<ComponentSymbol, Set<ComponentStreamUnitsSymbol>> availableStreams;
    private Set<String> testedComponents;
    private List<FileContent> files;
    private TestsMainEntryViewModel viewModelForMain;
    public static Set<String> availableComponents;

    TestsGeneratorCPP(GeneratorCPP generator) {
        this.generator = Log.errorIfNull(generator);
    }

    public List<FileContent> generateStreamTests(Scope symTab) {
        bluePrints = new ArrayList<>(generator.getBluePrints());
        findStreams(symTab);
        findComponents(symTab);

        if (bluePrints.isEmpty()) {
            Log.warn("no blue prints were generated");
            //return Collections.emptyList();
        }
        return generateFiles();
    }

    private void findStreams(Scope symTab) {
        StreamScanner scanner = new StreamScanner(generator.getModelsDirPath(), symTab);
        availableStreams = new HashMap<>(scanner.scan());
    }

    public void findComponents(Scope symTab) {
        ComponentScanner componentScanner = new ComponentScanner(generator.getModelsDirPath(), symTab, "emam");
        availableComponents = componentScanner.scan();
    }

    private List<FileContent> generateFiles() {
        testedComponents = new HashSet<>();
        files = new ArrayList<>();
        viewModelForMain = new TestsMainEntryViewModel();
        viewModelForMain.setIncludes(new ArrayList<>());
        for (BluePrintCPP b : bluePrints) {
            ExpandedComponentInstanceSymbol s = b.getOriginalSymbol();
            if (s != null) {
                processBluePrint(b, s);
            } else {
                Log.warn("no symbol info for blue print " + b.getName() + " (package: " + b.getPackageName() + ")");
            }
        }
        if (generator.isGenerateTests()) {
            files.add(new FileContent(AllTemplates.generateMainEntry(viewModelForMain), TESTS_DIRECTORY_NAME + "/tests_main.cpp"));
            files.add(getCatchLib());
        }
        //files.add(new FileContent(getTestedComponentsString(), TESTS_DIRECTORY_NAME + "/testedComponents.txt"));
        if (generator.isCheckModelDir()) {
            files.add(new FileContent(getExistingComponentStreamNames(), "/reporting/" + "existingStreams.txt"));
            files.add(new FileContent(getExistingComponentNames(), "/reporting/" + "existingComponents.txt"));
            files.add(new FileContent(getComponentNamesThatHaveTests(), "/reporting/" + "testComponents.txt"));
        }
        return files;
    }

    private String getExistingComponentNames() {
        String result = "Components:\n";
        for (String s : availableComponents) {
            result += "    " + s + "\n";
        }
        return result;
    }

    private String getExistingComponentStreamNames() {
        String result = "";
        for (ComponentSymbol k : availableStreams.keySet()) {
            result += "Streams for component " + k.getFullName() + ":\n";
            Iterator<ComponentStreamUnitsSymbol> iter = availableStreams.get(k).iterator();
            while (iter.hasNext()) {
                ComponentStreamUnitsSymbol cus = iter.next();
                result += "    " + cus.getFullName();
                result += "\n";
            }
        }
        return result;
    }

    private String getComponentNamesThatHaveTests() {
        String result = "";
        for (ComponentSymbol k : availableStreams.keySet()) {
            result += k.getFullName() + "\n";
        }
        return result;
    }

    private String getTestedComponentsString() {
        String result = "";
        for (String t : testedComponents) {
            result += t + "\n";
        }
        return result;
    }

    private void processBluePrint(BluePrintCPP b, ExpandedComponentInstanceSymbol s) {
        ComponentSymbol cs = s.getComponentType().getReferencedSymbol();
        if (testedComponents.add(cs.getFullName())) {
            processBluePrint(b, cs);
        }
    }

    private void processBluePrint(BluePrintCPP b, ComponentSymbol cs) {
        Set<ComponentStreamUnitsSymbol> streamsForComponent = availableStreams.get(cs);
        if (streamsForComponent == null || streamsForComponent.isEmpty()) {
            return;
        }
        //this.componentStreamNames.put(cs.getFullName(), streamsForComponent.toString());
        ComponentStreamTestViewModel viewModel = getStreamViewModel(b, cs, streamsForComponent);
        String genTestCode = AllTemplates.generateComponentStreamTest(viewModel);
        files.add(new FileContent(genTestCode, getFileName(viewModel)));
        viewModelForMain.getIncludes().add(viewModel.getFileNameWithExtension());

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
        Map<PortSymbol, ASTStream> port2NamedStream = getPort2NamedStream(cs, stream);
        int streamLength = getStreamLengths(port2NamedStream, stream);
        List<ComponentCheckViewModel> result = new ArrayList<>();
        for (int i = 0; i < streamLength; i++) {
            ComponentCheckViewModel vm = new ComponentCheckViewModel();
            vm.setInputPortName2Value(new HashMap<>());
            vm.setOutputPortName2Check(new HashMap<>());
            for (Map.Entry<PortSymbol, ASTStream> kv : port2NamedStream.entrySet()) {
                ASTStreamInstruction nextInstruction = kv.getValue().getStreamInstructions().get(i);
                processInstruction(vm, nextInstruction, kv.getKey());
            }
            result.add(vm);
        }
        return result;
    }

    private static Map<PortSymbol, ASTStream> getPort2NamedStream(ComponentSymbol cs, ComponentStreamUnitsSymbol stream) {
        Map<PortSymbol, ASTStream> port2NamedStream = new HashMap<>();
        for (PortSymbol port : cs.getPorts()) {
            NamedStreamUnitsSymbol namedStreamForPort = stream.getNamedStream(port.getName()).orElse(null);
            if (namedStreamForPort != null && namedStreamForPort.getAstNode().isPresent()) {
                ASTNamedStreamUnits node = (ASTNamedStreamUnits) namedStreamForPort.getAstNode().get();
                port2NamedStream.put(port, node.getStream());
            }
        }
        return port2NamedStream;
    }

    private static int getStreamLengths(Map<PortSymbol, ASTStream> port2NamedStream, ComponentStreamUnitsSymbol stream) {
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
        return streamLength;
    }

    private static void processInstruction(ComponentCheckViewModel vm, ASTStreamInstruction nextInstruction, PortSymbol port) {
        if (nextInstruction.getStreamValue().isPresent()) {
            ASTStreamValue sv = nextInstruction.getStreamValue().get();
            String portName = port.getName();
            if (port.isIncoming()) {
                processIncomingPort(vm, sv, portName);
            } else {
                processOutgoingPort(vm, sv, portName);
            }
        }
    }

    private static void processIncomingPort(ComponentCheckViewModel vm, ASTStreamValue sv, String portName) {
        ASTStreamValue2InputPortValue converter = new ASTStreamValue2InputPortValue();
        sv.accept(converter);
        if (converter.getResult() != null) {
            vm.getInputPortName2Value().put(portName, converter.getResult());
        }
    }

    private static void processOutgoingPort(ComponentCheckViewModel vm, ASTStreamValue sv, String portName) {
        ASTStreamValue2OutputPortCheck converter = new ASTStreamValue2OutputPortCheck();
        sv.accept(converter);
        if (converter.getResult() != null) {
            vm.getOutputPortName2Check().put(portName, converter.getResult());
        }
    }

    private static FileContent getCatchLib() {
        return FileUtil.getResourceAsFile(
                "/vendor/catch.hpp",
                TESTS_DIRECTORY_NAME + "/catch.hpp"
        );
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
