/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.ComponentScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.StreamScanner;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ComponentStreamTestViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.StreamViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.TestsMainEntryViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.BooleanOutputPortCheck;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.ComponentCheckViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.IOutputPortCheck;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.RangeOutputPortCheck;
import de.monticore.lang.monticar.streamunits._ast.*;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.monticar.streamunits._symboltable.NamedStreamUnitsSymbol;
import de.monticore.lang.monticar.streamunits._visitor.StreamUnitsVisitor;
import de.monticore.literals.literals._ast.ASTBooleanLiteral;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public final class TestsGeneratorCPP {

    public static final String TESTS_DIRECTORY_NAME = "/test";

    private final GeneratorCPP generator;
    private List<BluePrintCPP> bluePrints;
    public static Map<EMAComponentSymbol, Set<ComponentStreamUnitsSymbol>> availableStreams;
    private Set<String> testedComponents;
    private List<FileContent> files;
    private TestsMainEntryViewModel viewModelForMain;
    public static Set<String> availableComponents;

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
        for (BluePrintCPP b : bluePrints) {
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
                "                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})";
        cmake.addCMakeCommandEnd(executeTestTplt.replace("<name>",compName));
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

    private void processBluePrint(BluePrintCPP b, EMAComponentInstanceSymbol s) {
        //TODO: 123 : check if EMAComponentSymbol is correct choice here:  ComponentSymbol cs = s.getComponentType().getReferencedSymbol();
        EMAComponentSymbol cs = s.getComponentType().getReferencedSymbol();
        if (testedComponents.add(cs.getFullName())) {
            processBluePrint(b, cs);
        }
    }

    private void processBluePrint(BluePrintCPP b, EMAComponentSymbol cs) {
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

    private static ComponentStreamTestViewModel getStreamViewModel(BluePrintCPP b, EMAComponentSymbol cs, Set<ComponentStreamUnitsSymbol> streamsForComponent) {
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
        Map<EMAPortSymbol, ASTStream> port2NamedStream = getPort2NamedStream(cs, stream);
        int streamLength = getStreamLengths(port2NamedStream, stream);
        List<ComponentCheckViewModel> result = new ArrayList<>();
        for (int i = 0; i < streamLength; i++) {
            ComponentCheckViewModel vm = new ComponentCheckViewModel();
            vm.setInputPortName2Value(new HashMap<>());
            vm.setOutputPortName2Check(new HashMap<>());
            for (Map.Entry<EMAPortSymbol, ASTStream> kv : port2NamedStream.entrySet()) {
                ASTStreamInstruction nextInstruction = kv.getValue().getStreamInstructionList().get(i);
                processInstruction(vm, nextInstruction, kv.getKey());
            }
            result.add(vm);
        }
        return result;
    }

    private static Map<EMAPortSymbol, ASTStream> getPort2NamedStream(EMAComponentSymbol cs, ComponentStreamUnitsSymbol stream) {
        Map<EMAPortSymbol, ASTStream> port2NamedStream = new HashMap<>();
        for (EMAPortSymbol port : cs.getPortsList()) {
            NamedStreamUnitsSymbol namedStreamForPort = stream.getNamedStream(port.getName()).orElse(null);
            if (namedStreamForPort != null && namedStreamForPort.getAstNode().isPresent()) {
                ASTNamedStreamUnits node = (ASTNamedStreamUnits) namedStreamForPort.getAstNode().get();
                port2NamedStream.put(port, node.getStream());
            }
        }
        return port2NamedStream;
    }

    private static int getStreamLengths(Map<EMAPortSymbol, ASTStream> port2NamedStream, ComponentStreamUnitsSymbol stream) {
        int streamLength = -1;
        for (ASTStream ns : port2NamedStream.values()) {
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

    private static void processInstruction(ComponentCheckViewModel vm, ASTStreamInstruction nextInstruction, EMAPortSymbol port) {
        if (nextInstruction.getStreamValueOpt().isPresent()) {
            ASTStreamValue sv = nextInstruction.getStreamValueOpt().get();
            String portName = port.getName();
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
                } else if (node.getValuePairOpt().isPresent()) {
                    //TODO valuepair conversion
                    builder.append("NOT HANDLED VALUEPAIROPT!!!");
                }
                Log.debug("Result: " + builder.toString(), "TestGeneratorCPP");
                result = RangeOutputPortCheck.from(builder.toString(), builder.toString(),true);
            }
            handled = true;
        }
    }


    private static final class ASTStreamValue2InputPortValue implements StreamUnitsVisitor {

        private String result = null;
        boolean handled = false;

        public String getResult() {
            return result;
        }

        @Override
        public void visit(ASTBooleanLiteral node) {

            if (!handled) result = "=" + (node.getValue() ? "true" : "false");
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
                } else if (node.getValuePairOpt().isPresent()) {
                    //TODO valuepair conversion
                    result += "NOT HANDLED VALUEPAIROPT!!!";
                }
                Log.debug("Result: " + builder.toString(), "TestGeneratorCPP.vists(ASTStreamArrayValues)");
                result += builder.toString();
            }
            handled = true;
        }
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
}
