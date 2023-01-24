/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.ComponentScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.StreamScanner;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ComponentStreamTestViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.StreamViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.TestsMainEntryViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.*;
import de.monticore.lang.monticar.streamunits._ast.*;
import de.monticore.lang.monticar.streamunits._parser.StreamUnitsParser;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.monticar.streamunits._symboltable.NamedStreamUnitsSymbol;
import de.monticore.lang.monticar.streamunits._visitor.StreamUnitsVisitor;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.literals.literals._ast.ASTBooleanLiteral;
import de.monticore.literals.literals._ast.ASTDoubleLiteral;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.Scope;
import de.monticore.types.types._ast.ASTType;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FilenameUtils;

import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

public final class TestsGeneratorCPP {

    public static final String TESTS_DIRECTORY_NAME = "/test";

    private final GeneratorCPP generator;
    private List<EMAMBluePrintCPP> bluePrints;
    public static Map<EMAComponentSymbol, Set<ComponentStreamUnitsSymbol>> availableStreams;
    private Set<String> testedComponents;
    private List<FileContent> files;
    private TestsMainEntryViewModel viewModelForMain;
    public static Set<String> availableComponents;
    private static boolean useOpenCV = false;

    TestsGeneratorCPP(GeneratorCPP generator) {
        this.generator = Log.errorIfNull(generator);
    }

    public List<FileContent> generateStreamTests(Scope symTab, EMAComponentInstanceSymbol componentSymbol) {
        bluePrints = new ArrayList<>(generator.getBluePrints());
        findStreams(symTab);
        findComponents(symTab);

        if (bluePrints.isEmpty()) {
            Log.warn("no blue prints were generated");
            //return Collections.emptyList();
        }
        return generateFiles(componentSymbol);
    }

    private void findStreams(Scope symTab) {
        StreamScanner scanner = new StreamScanner(generator.getModelsDirPath(), symTab);
        availableStreams = new HashMap<>(scanner.scan());
    }

    public void findComponents(Scope symTab) {
        ComponentScanner componentScanner = new ComponentScanner(generator.getModelsDirPath(), symTab, "emam");
        availableComponents = componentScanner.scan();
    }

    private List<FileContent> generateFiles(EMAComponentInstanceSymbol componentSymbol) {
        testedComponents = new HashSet<>();
        files = new ArrayList<>();
        viewModelForMain = new TestsMainEntryViewModel();
        viewModelForMain.setIncludes(new ArrayList<>());
        for (EMAMBluePrintCPP b : bluePrints) {
            EMAComponentInstanceSymbol s = b.getOriginalSymbol();
            if (s != null) {
                processBluePrint(b, s);
            } else {
                Log.warn("no symbol info for blue print " + b.getName() + " (package: " + b.getPackageName() + ")");
            }
        }
        if (generator.isGenerateTests()) {
            boolean isOctaveBackend = true;
            if (!MathConverter.curBackend.getBackendName().equals(OctaveBackend.NAME))
                isOctaveBackend = false;
            files.add(new FileContent(AllTemplates.generateMainEntry(viewModelForMain, isOctaveBackend), TESTS_DIRECTORY_NAME + "/tests_main.cpp"));
        }
        //files.add(new FileContent(getTestedComponentsString(), TESTS_DIRECTORY_NAME + "/testedComponents.txt"));
        if (generator.isCheckModelDir()) {
            files.add(new FileContent(getExistingComponentStreamNames(), "/reporting/" + "existingStreams.txt"));
            files.add(new FileContent(getExistingComponentNames(), "/reporting/" + "existingComponents.txt"));
            files.add(new FileContent(getComponentNamesThatHaveTests(), "/reporting/" + "testComponents.txt"));
        }
        // add to cmake lists
        if (generator.isGenerateCMakeEnabled())
            addTestExecutionToCMakeConfig(componentSymbol);
        return files;
    }

    private void addTestExecutionToCMakeConfig(EMAComponentInstanceSymbol componentSymbol) {
        // executable name
        String compName;
        final String execuatablePostFix = "_StreamTests";
        CMakeConfig cmake = generator.getCMakeConfig();
        cmake.addCMakeCommandEnd("include_directories(test)");
        if (componentSymbol != null) {
            compName = componentSymbol.getFullName().replace('.', '_').replace('[', '_').replace(']', '_');
        } else {
            compName = "";
        }
        cmake.addCMakeCommandEnd("add_executable(" + compName + execuatablePostFix + "  test/tests_main.cpp)");
        cmake.addCMakeCommandEnd("target_compile_definitions(" + compName + execuatablePostFix + " PRIVATE CATCH_CONFIG_MAIN=1 ARMA_DONT_USE_WRAPPER)");
        if (compName.isEmpty()) {
            cmake.addCMakeCommandEnd("target_include_directories(" + compName + execuatablePostFix + "  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})");
            cmake.addCMakeCommandEnd("target_link_libraries(" + compName + execuatablePostFix + "  PUBLIC ${LIBS})");
        } else { // link against already created static library
            cmake.addCMakeCommandEnd("target_link_libraries(" + compName + execuatablePostFix + "  PUBLIC " + compName + ")");
        }
        cmake.addCMakeCommandEnd("set_target_properties(" + compName + execuatablePostFix + "  PROPERTIES LINKER_LANGUAGE CXX)");

        String executeTestTplt = "\n# execute tests\n" +
                "add_custom_target(run_<name>_StreamTests ALL\n" +
                "                  COMMAND <name>_StreamTests\n" +
                "                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})";
        cmake.addCMakeCommandEnd(executeTestTplt.replace("<name>", compName));
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
        for (EMAComponentSymbol k : availableStreams.keySet()) {
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
        for (EMAComponentSymbol k : availableStreams.keySet()) {
            result += getSmallStartingName(k.getFullName()) + "\n";
        }
        return result;
    }

    private String getSmallStartingName(String name) {
        String result = "";
        String splits[] = name.split("\\.");

        for (int i = 0; i < splits.length - 1; ++i) {
            result += splits[i] + ".";
        }

        result += getStringFirstLetterSmall(splits[splits.length - 1]);
        return result;
    }

    private String getStringFirstLetterSmall(String name) {
        String result = "";
        String firstLetter = "" + name.charAt(0);
        firstLetter = firstLetter.toLowerCase();
        result += firstLetter;
        result += name.substring(1);
        return result;
    }

    private String getTestedComponentsString() {
        String result = "";
        for (String t : testedComponents) {
            result += t + "\n";
        }
        return result;
    }

    private void processBluePrint(EMAMBluePrintCPP b, EMAComponentInstanceSymbol s) {
        //TODO: 123 : check if EMAComponentSymbol is correct choice here:  ComponentSymbol cs = s.getComponentType().getReferencedSymbol();
        if (s.getComponentType() == null) return;
        EMAComponentSymbol cs = s.getComponentType().getReferencedSymbol();
        if (testedComponents.add(cs.getFullName())) {
            processBluePrint(b, cs);
        }
    }

    private void processBluePrint(EMAMBluePrintCPP b, EMAComponentSymbol cs) {
        Set<ComponentStreamUnitsSymbol> streamsForComponent = availableStreams.get(cs);
        if (streamsForComponent == null || streamsForComponent.isEmpty()) {
            return;
        }
        //this.componentStreamNames.put(cs.getFullName(), streamsForComponent.toString());
        ComponentStreamTestViewModel viewModel = getStreamViewModel(b, cs, streamsForComponent);
        viewModel.setUseOpenCV(this.isUseOpenCV());
        String genTestCode = AllTemplates.generateComponentStreamTest(viewModel);
        files.add(new FileContent(genTestCode, getFileName(viewModel)));
        viewModelForMain.getIncludes().add(viewModel.getFileNameWithExtension());

    }

    private static ComponentStreamTestViewModel getStreamViewModel(EMAMBluePrintCPP b, EMAComponentSymbol cs, Set<ComponentStreamUnitsSymbol> streamsForComponent) {
        ComponentStreamTestViewModel viewModel = new ComponentStreamTestViewModel();
        viewModel.setComponentName(b.getName());
        viewModel.setFileNameWithoutExtension(b.getName() + "_test");
        viewModel.setStreams(new ArrayList<>());
        for (ComponentStreamUnitsSymbol stream : streamsForComponent) {
            StreamViewModel svm = new StreamViewModel();

            Collection<EMAPortSymbol> outPorts = cs.getOutgoingPorts();
            List<String> names = new ArrayList<>();
            for (EMAPortSymbol portSymbol : outPorts) {
                names.add(portSymbol.getName());
            }
            svm.outputPortNames = names;

            viewModel.getStreams().add(svm);
            svm.setName(stream.getFullName());
            svm.setChecks(getComponentPortChecks(cs, stream));
        }
        return viewModel;
    }

    private static List<ComponentCheckViewModel> getComponentPortChecks(EMAComponentSymbol cs, ComponentStreamUnitsSymbol stream) {
        List<PortStreamTuple> port2NamedStream = getPort2NamedStream(cs, stream);
        int streamLength = getStreamLengths(port2NamedStream, stream);
        List<ComponentCheckViewModel> result = new ArrayList<>();
        for (int i = 0; i < streamLength; i++) {
            ComponentCheckViewModel vm = new ComponentCheckViewModel();
            vm.setInputPortName2Value(new HashMap<>());
            vm.setOutputPortName2Check(new HashMap<>());
            for (PortStreamTuple tuple : port2NamedStream) {
                ASTStreamInstruction nextInstruction = tuple.getNamedStreamUnits().getStream().getStreamInstructionList().get(i);
                processInstruction(vm, nextInstruction, tuple);
            }
            result.add(vm);
        }
        return result;
    }

    private static List<PortStreamTuple> getPort2NamedStream(EMAComponentSymbol cs, ComponentStreamUnitsSymbol stream) {
        List<PortStreamTuple> port2NamedStream = new ArrayList<>();
        for (EMAPortSymbol port : cs.getPortsList()) {


//            NamedStreamUnitsSymbol namedStreamForPort = stream.getNamedStream(port.getName()).orElse(null);
            List<NamedStreamUnitsSymbol> namedStreamForPortList = stream.getNamedStreams();
            for (NamedStreamUnitsSymbol namedStreamForPort : namedStreamForPortList) {
                boolean matchingName = (namedStreamForPort.getName().startsWith(port.getName() + ".") || namedStreamForPort.getName().equals(port.getName()));
                if (matchingName && namedStreamForPort.getAstNode().isPresent()) {
                    ASTNamedStreamUnits node = (ASTNamedStreamUnits) namedStreamForPort.getAstNode().get();
                    port2NamedStream.add(new PortStreamTuple(port, node));
                }
            }
        }
        return port2NamedStream;
    }

    private static int getStreamLengths(List<PortStreamTuple> port2NamedStream, ComponentStreamUnitsSymbol stream) {
        int streamLength = -1;
        for (PortStreamTuple tuple : port2NamedStream) {
            ASTStream ns = tuple.getNamedStreamUnits().getStream();
            int l = ns.getStreamInstructionList().size();
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

    private static void processInstruction(ComponentCheckViewModel vm, ASTStreamInstruction nextInstruction, PortStreamTuple portStreamTuple) {
        EMAPortSymbol port = portStreamTuple.getPort();

        if (nextInstruction.getStreamValueOpt().isPresent()) {
            ASTStreamValue sv = nextInstruction.getStreamValueOpt().get();
            String portName;
            ASTNamedStreamUnits namedStreamUnits = portStreamTuple.getNamedStreamUnits();
            if(namedStreamUnits.isEmptyFieldQualifiers()){
                portName = port.getName();
            }else{
                portName = port.getName() + "." + String.join(".", namedStreamUnits.getFieldQualifierList());
            }
            if (port.isIncoming()) {
                processIncomingPort(vm, sv, portName);
            } else {
                processOutgoingPort(vm, sv, portName);
            }
        } else if (nextInstruction.getStreamArrayValuesOpt().isPresent()) {
            ASTStreamArrayValues sv = nextInstruction.getStreamArrayValues();
            String portName = port.getName();
            if (port.isIncoming()) {
                processIncomingPortArray(vm, sv, portName);
            } else {
                processOutgoingPortArray(vm, sv, portName);
            }
        } else if (nextInstruction.getStreamCompareOpt().isPresent()) {
            Log.error("Not handled!");
        } else if (nextInstruction.getFilePathOpt().isPresent()) {
            processFilePath(vm, nextInstruction.getFilePathOpt().get(), portStreamTuple, port);
        }
    }

    private static void processInstructionForFilePath(ComponentCheckViewModel vm,
                                                      ASTStreamInstruction nextInstruction,
                                                      PortStreamTuple portStreamTuple,
                                                      Optional<ASTDoubleLiteral> elementTolerance,
                                                      Optional<ASTDoubleLiteral> generalTolerance
                                                      ) {
        EMAPortSymbol port = portStreamTuple.getPort();

        if (nextInstruction.getStreamValueOpt().isPresent()) {
            ASTStreamValue sv = nextInstruction.getStreamValueOpt().get();
            String portName;
            ASTNamedStreamUnits namedStreamUnits = portStreamTuple.getNamedStreamUnits();
            if(namedStreamUnits.isEmptyFieldQualifiers()){
                portName = port.getName();
            }else{
                portName = port.getName() + "." + String.join(".", namedStreamUnits.getFieldQualifierList());
            }
            if (port.isIncoming()) {
                processIncomingPort(vm, sv, portName);
            } else {
                processOutgoingPort(vm, sv, portName);
            }
        } else if (nextInstruction.getStreamArrayValuesOpt().isPresent()) {
            ASTStreamArrayValues sv = nextInstruction.getStreamArrayValues();
            if (elementTolerance.isPresent()) sv.setElementToleranceOpt(elementTolerance);
            if (generalTolerance.isPresent()) sv.setGeneralToleranceOpt(generalTolerance);
            String portName = port.getName();
            if (port.isIncoming()) {
                processIncomingPortArray(vm, sv, portName);
            } else {
                processOutgoingPortArray(vm, sv, portName);
            }
        } else if (nextInstruction.getStreamCompareOpt().isPresent()) {
            Log.error("Not handled!");
        } else if (nextInstruction.getFilePathOpt().isPresent()) {
            processFilePath(vm, nextInstruction.getFilePathOpt().get(), portStreamTuple, port);
        }
    }

    private static Optional<List<Integer>> getInputSize(EMAPortSymbol portSymbol) {
        List<Integer> size = new ArrayList<>();
        if (portSymbol.getTypeReference() instanceof MCASTTypeSymbol){
            ASTType typeSymbol = ((MCASTTypeSymbol) portSymbol.getTypeReference()).getAstType();
            if (typeSymbol instanceof ASTCommonMatrixType) {
                List<ASTExpression> dimensionsList = ((ASTCommonMatrixType) typeSymbol).getDimension().getDimensionList();
                for (ASTExpression ex : dimensionsList) {
                    if (ex instanceof ASTNumberExpression) {
                        ((ASTNumberExpression) ex).getNumberWithUnit().getNumber().ifPresent(aDouble -> size.add(aDouble.intValue()));
                    } else {
                        return Optional.empty();
                    }
                }
                return Optional.of(size);
            }
        }
        return Optional.empty();
    }

    private static void processFilePath(ComponentCheckViewModel vm, ASTFilePath astFilePath, PortStreamTuple portStreamTuple, EMAPortSymbol port) {
        final String dir = System.getProperty("user.dir");
        final String filePath = dir + astFilePath.getStringLiteral().getSource();
        File file = new File(filePath);

        if (file.exists()) {
            try {
                handleFileByExtension(file, port, vm, astFilePath, portStreamTuple);
            } catch (IOException | NumberFormatException e) {
                if (e instanceof  IOException) {
                    Log.error("Error on reading file:" + filePath);
                } else {
                    e.printStackTrace();
                }
            }
        } else {
            Log.error("File: " + filePath + " does not exist!");
        }

    }

    private static void handleFileByExtension(File file, EMAPortSymbol port,
                                                                        ComponentCheckViewModel vm, ASTFilePath astFilePath,
                                                                        PortStreamTuple portStreamTuple) throws IOException {
        String extension = FilenameUtils.getExtension(file.getPath());
        if (extension.equals("txt")) {
            Path path = file.toPath();
            List<String> content = Files.readAllLines(path, Charset.defaultCharset());
            Optional<ASTStreamInstruction> astStreamInstruction = new StreamUnitsParser().parse_StringStreamInstruction((content.get(0)));
            astStreamInstruction.ifPresent(instruction ->
            {processInstructionForFilePath(vm, instruction,
                    portStreamTuple, astFilePath.getElementToleranceOpt(), astFilePath.getGeneralToleranceOpt());});
        } else if (extension.equals("png")) {
            useOpenCV = true;
            if (port.isIncoming()) {
                processImagePathIncomingPort(vm, astFilePath, port);
            } else {
                processImagePathOutgoingPort(vm, astFilePath, port);
            }
        } else {
            Log.error("File Extension not supported");
        }
    }

    private static void processImagePathIncomingPort(ComponentCheckViewModel vm, ASTFilePath imagePath, EMAPortSymbol port) {
        final String dir = System.getProperty("user.dir");
        final String filePath = dir + imagePath.getStringLiteral().getValue();
        File file = new File(filePath);

        if (file.exists()) {
            ASTStreamValue2InputPortValue converter = new ASTStreamValue2InputPortValue();
            imagePath.accept(converter);
            if (converter.getResult() != null) {
                InputPort result = converter.getResult();
                Optional<List<Integer>> sizeOpt = getInputSize(port);
                if (sizeOpt.isPresent()) {
                    List<Integer> size = sizeOpt.get();
                    for (int i = 0; i < size.size(); i++) {
                        switch (i) {
                            case 0:
                                result.setN_slices(size.get(0));
                                break;
                            case 1:
                                result.setN_rows(size.get(1));
                                break;
                            case 2:
                                result.setN_cols(size.get(2));
                                break;
                        }
                    }
                }
                vm.getInputPortName2Value().put(port.getName(), result);
            }
        } else {
            Log.error("File " + file + " not exists");
        }
    }

    private static void processImagePathOutgoingPort(ComponentCheckViewModel vm, ASTFilePath imagePath, EMAPortSymbol port) {
        final String dir = System.getProperty("user.dir");
        final String filePath = dir + imagePath.getStringLiteral().getValue();
        File file = new File(filePath);

        if (file.exists()) {
            ASTStreamValue2OutputPortCheck converter = new ASTStreamValue2OutputPortCheck(port.getName());
            imagePath.accept(converter);
            if (converter.getResult() != null) {
                RangeOutputPortCheck result = (RangeOutputPortCheck) converter.getResult();
                Optional<List<Integer>> sizeOpt = getInputSize(port);
                if (sizeOpt.isPresent()) {
                    List<Integer> size = sizeOpt.get();
                    for (int i = 0; i < size.size(); i++) {
                        switch (i) {
                            case 0:
                                result.setN_slices(size.get(0));
                                break;
                            case 1:
                                result.setN_rows(size.get(1));
                                break;
                            case 2:
                                result.setN_cols(size.get(2));
                                break;
                        }
                    }
                }
                vm.getOutputPortName2Check().put(port.getName(), result);
            }
        } else {
            Log.error("File " + file + " not exists");
        }
    }


    private static void processIncomingPort(ComponentCheckViewModel vm, ASTStreamValue sv, String portName) {
        ASTStreamValue2InputPortValue converter = new ASTStreamValue2InputPortValue();
        sv.accept(converter);
//        System.out.println("Processing: " + portName);
        Log.debug("Processing: " + portName, "TestGeneratorCPP.processIncomingPort");
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

    private static void processIncomingPortArray(ComponentCheckViewModel vm, ASTStreamArrayValues sv, String portName) {
        ASTStreamValue2InputPortValue converter = new ASTStreamValue2InputPortValue();
        sv.accept(converter);
        Log.debug("Processing: " + portName, "TestGeneratorCPP.processIncomingPortArray");
        if (converter.getResult() != null) {
            vm.getInputPortName2Value().put(portName, converter.getResult());
        }
    }

    private static void processOutgoingPortArray(ComponentCheckViewModel vm, ASTStreamArrayValues sv, String portName) {
        ASTStreamValue2OutputPortCheck converter = new ASTStreamValue2OutputPortCheck(portName);
        sv.accept(converter);
        if (converter.getResult() != null) {
            vm.getOutputPortName2Check().put(portName, converter.getResult());
        }
    }

    private static String getFileName(ComponentStreamTestViewModel viewModel) {
        return TESTS_DIRECTORY_NAME + "/" + viewModel.getFileNameWithExtension();
    }

    public boolean isUseOpenCV() {
        return useOpenCV;
    }

    public void setUseOpenCV(boolean useOpenCV) {
        this.useOpenCV = useOpenCV;
    }

    private static final class ASTStreamValue2OutputPortCheck implements StreamUnitsVisitor {


        private IOutputPortCheck result = null;

        public IOutputPortCheck getResult() {
            return result;
        }

        boolean handled = false;
        String portName = "";
        boolean isMatrix = false;

        public boolean getIsMatrix() {
            return isMatrix;
        }

        public ASTStreamValue2OutputPortCheck(String portName) {
            this.portName = portName;
        }

        public ASTStreamValue2OutputPortCheck() {
        }

        @Override
        public void visit(ASTBooleanLiteral node) {
            if (!handled) {
                if (node.getValue()) {
                    result = BooleanOutputPortCheck.TRUE_EXPECTED;
                } else {
                    result = BooleanOutputPortCheck.FALSE_EXPECTED;
                }
            }
            handled = true;
        }

        @Override
        public void visit(ASTPrecisionNumber node) {
            if (!handled) {
                ASTNumberWithUnit unitNumber = node.getNumberWithUnit();
                if (!unitNumber.getNumber().isPresent()) {
                    return;
                }
                double baseValue = unitNumber.getNumber().get().doubleValue();
                if (node.getPrecisionOpt().isPresent()
                        && node.getPrecisionOpt().get().getNumberWithUnit().getNumber().isPresent()) {
                    double delta = node.getPrecisionOpt().get().getNumberWithUnit().getNumber().get().doubleValue();
                    result = RangeOutputPortCheck.from(baseValue - delta, baseValue + delta);
                } else {
                    result = RangeOutputPortCheck.from(baseValue, baseValue);
                }
            }
            handled = true;
        }

        @Override
        public void visit(ASTFilePath node) {
            if (!handled) {
                double elementTolerance = node.getElementToleranceOpt().isPresent() ? node.getElementTolerance().getValue() : 0;
                double generalTolerance = node.getGeneralToleranceOpt().isPresent() ? node.getGeneralTolerance().getValue() : 0;
                result = RangeOutputPortCheck.from(node.getStringLiteral().getValue(), elementTolerance, generalTolerance);
            }
            handled = true;
        }

        @Override
        public void visit(ASTStreamArrayValues node) {
            if (!handled) {
                isMatrix = true;
                StringBuilder builder = new StringBuilder();
                StreamValueConverter converter = new StreamValueConverter();
                if (node.getMatrixPairOpt().isPresent()) {
                    builder.append(" <<");
                    for (int j = 0; j < node.getMatrixPair().getValuePairList().size(); ++j) {
                        ASTValuePair valuePair = node.getMatrixPair().getValuePair(j);
                        for (int i = 0; i < valuePair.getStreamValueList().size(); ++i) {
                            ASTStreamValue value = valuePair.getStreamValueList().get(i);
                            //TODO Name, PrecisionNumber, SignedLiteral, valueAtTick
                            if (value.getNameOpt().isPresent()) {
                                builder.append(value.getName());
                            } else {
                                builder.append(converter.convert(value));
                            }
                            if (i + 1 < valuePair.getStreamValueList().size()) {
                                builder.append(" << ");
                            }
                        }
                        builder.append("<< arma::endr ");

                        if (j + 1 < node.getMatrixPair().getValuePairList().size()) {
                            builder.append(" << ");
                        }
                    }
                    Log.debug("Result: " + builder.toString(), "TestGeneratorCPP");
                    result = RangeOutputPortCheck.from(builder.toString(), builder.toString(), true);
                } else if (node.getValuePairOpt().isPresent()) {
                    convertValuePair(builder, converter, node.getValuePair());
                    Log.debug("Result: " + builder.toString(), "TestGeneratorCPP");
                    result = RangeOutputPortCheck.from(builder.toString(), builder.toString(), true);
                } else if (node.getCubePairOpt().isPresent()) {
                    List<String> boundList = new ArrayList<>();
                    for (int k = 0; k < node.getCubePair().getMatrixPairList().size(); k++) {
                        ASTMatrixPair matrixPair = node.getCubePair().getMatrixPairList().get(k);
                        builder.append(" <<");
                        for (int j = 0; j < matrixPair.getValuePairList().size(); ++j) {
                            ASTValuePair valuePair = matrixPair.getValuePair(j);
                            for (int i = 0; i < valuePair.getStreamValueList().size(); ++i) {
                                ASTStreamValue value = valuePair.getStreamValueList().get(i);
                                if (value.getNameOpt().isPresent()) {
                                    builder.append(value.getName());
                                } else {
                                    builder.append(converter.convert(value));
                                }
                                if (i + 1 < valuePair.getStreamValueList().size()) {
                                    builder.append(" << ");
                                }
                            }
                            builder.append("<< arma::endr ");

                            if (j + 1 < matrixPair.getValuePairList().size()) {
                                builder.append(" << ");
                            }
                        }
                        boundList.add(builder.toString());
                        builder.setLength(0);
                    }
                    double elementTolerance = node.getElementToleranceOpt().isPresent() ? node.getElementTolerance().getValue() : 0;
                    double generalTolerance = node.getGeneralToleranceOpt().isPresent() ? node.getGeneralTolerance().getValue() : 0;
                    isMatrix = true;
                    result = RangeOutputPortCheck.from(boundList,
                            boundList,
                            node.getCubePair().sizeMatrixPairs(),
                            node.getCubePair().getMatrixPairList().get(0).sizeValuePairs(),
                            node.getCubePair().getMatrixPairList().get(0).getValuePairList().get(0).sizeStreamValues(),
                            elementTolerance, generalTolerance);

                }

            }
            handled = true;
        }
    }


    private static final class ASTStreamValue2InputPortValue implements StreamUnitsVisitor {

        private String result = "";
        boolean handled = false;
        boolean isCube = false;

        private InputPort inputPort;

        public ASTStreamValue2InputPortValue() {}

        public InputPort getResult() {
            return inputPort;
        }

        @Override
        public void visit(ASTBooleanLiteral node) {

            if (!handled) {
                result = "=" + (node.getValue() ? "true" : "false");
                inputPort = InputPort.from(result);
            }
            handled = true;
        }

        @Override
        public void visit(ASTPrecisionNumber node) {
            if (!handled) {
                ASTNumberWithUnit unitNumber = node.getNumberWithUnit();
                if (!unitNumber.getNumber().isPresent()) {
                    return;
                }
                result = "= " + Double.toString(unitNumber.getNumber().get().doubleValue());
                inputPort = InputPort.from(result);
            }
            handled = true;
        }

        @Override
        public void visit(ASTFilePath node) {
            if (!handled) {
                inputPort = InputPort.from(node.getStringLiteral().getValue(), true);
            }
            handled = true;
        }

        @Override
        public void visit(ASTStreamArrayValues node) {
            if (!handled) {
                StringBuilder builder = new StringBuilder();
                StreamValueConverter converter = new StreamValueConverter();
                if (node.getMatrixPairOpt().isPresent()) {
                    result = " <<";
                    for (int j = 0; j < node.getMatrixPair().getValuePairList().size(); ++j) {
                        ASTValuePair valuePair = node.getMatrixPair().getValuePair(j);
                        for (int i = 0; i < valuePair.getStreamValueList().size(); ++i) {
                            ASTStreamValue value = valuePair.getStreamValueList().get(i);
                            //TODO Name, PrecisionNumber, SignedLiteral, valueAtTick
                            if (value.getNameOpt().isPresent()) {
                                builder.append(value.getName());
                            } else {
                                builder.append(converter.convert(value));
                            }
                            if (i + 1 < valuePair.getStreamValueList().size()) {
                                builder.append(" << ");
                            }
                        }
                        builder.append("<< arma::endr ");

                        if (j + 1 < node.getMatrixPair().getValuePairList().size()) {
                            builder.append(" << ");
                        }
                    }
                    Log.debug("Result: " + builder.toString(), "TestGeneratorCPP.vists(ASTStreamArrayValues)");
                    result += builder.toString();
                    inputPort = InputPort.from(result);
                } else if (node.getValuePairOpt().isPresent()) {
                    convertValuePair(builder, converter, node.getValuePair());
                    Log.debug("Result: " + builder.toString(), "TestGeneratorCPP.vists(ASTStreamArrayValues)");
                    result += builder.toString();
                    inputPort = InputPort.from(result);
                } else if (node.getCubePairOpt().isPresent()) {
                    isCube = true;
                    List<String> resultList = new ArrayList<>();
                    for (int k = 0; k < node.getCubePair().getMatrixPairList().size(); k++) {
                        ASTMatrixPair matrixPair = node.getCubePair().getMatrixPairList().get(k);
                        builder.append(" <<");
                        for (int j = 0; j < matrixPair.getValuePairList().size(); ++j) {
                            ASTValuePair valuePair = matrixPair.getValuePair(j);
                            for (int i = 0; i < valuePair.getStreamValueList().size(); ++i) {
                                ASTStreamValue value = valuePair.getStreamValueList().get(i);
                                //TODO Name, PrecisionNumber, SignedLiteral, valueAtTick
                                if (value.getNameOpt().isPresent()) {
                                    builder.append(value.getName());
                                } else {
                                    builder.append(converter.convert(value));
                                }
                                if (i + 1 < valuePair.getStreamValueList().size()) {
                                    builder.append(" << ");
                                }
                            }
                            builder.append("<< arma::endr ");

                            if (j + 1 < matrixPair.getValuePairList().size()) {
                                builder.append(" << ");
                            }
                        }
                        resultList.add(builder.toString());
                        builder.setLength(0);
                    }
                    inputPort = InputPort.from(resultList,
                            node.getCubePair().sizeMatrixPairs(),
                            node.getCubePair().getMatrixPairList().get(0).sizeValuePairs(),
                            node.getCubePair().getMatrixPairList().get(0).getValuePairList().get(0).sizeStreamValues());
                }

            }
            handled = true;
        }
    }

    private static void convertValuePair(StringBuilder builder, StreamValueConverter converter, ASTValuePair valuePair) {
        builder.append(" <<");
        for (int i = 0; i < valuePair.getStreamValueList().size(); ++i) {
            ASTStreamValue value = valuePair.getStreamValueList().get(i);
            //TODO Name, PrecisionNumber, SignedLiteral, valueAtTick
            if (value.getNameOpt().isPresent()) {
                builder.append(value.getName());
            } else {
                builder.append(converter.convert(value));
            }
            if (i + 1 < valuePair.getStreamValueList().size()) {
                builder.append(" << ");
            }
        }
        builder.append("<< arma::endr ");
    }

    private static class StreamValueConverter implements StreamUnitsVisitor {
        String result = "";

        public String convert(ASTStreamValue node) {
            result = "";
            handle(node);
            return result;
        }

        @Override
        public void visit(ASTBooleanLiteral node) {
            result = node.getValue() ? "true" : "false";
        }

        @Override
        public void visit(ASTPrecisionNumber node) {
            ASTNumberWithUnit unitNumber = node.getNumberWithUnit();
            if (!unitNumber.getNumber().isPresent()) {
                return;
            }
            result = Double.toString(unitNumber.getNumber().get().doubleValue());
        }

    }

    private static class PortStreamTuple {
        private EMAPortSymbol port;
        private ASTNamedStreamUnits namedStreamUnits;


        public PortStreamTuple(EMAPortSymbol port, ASTNamedStreamUnits namedStreamUnits) {
            this.port = port;
            this.namedStreamUnits = namedStreamUnits;
        }

        public EMAPortSymbol getPort() {
            return port;
        }

        public ASTNamedStreamUnits getNamedStreamUnits() {
            return namedStreamUnits;
        }
    }
}
