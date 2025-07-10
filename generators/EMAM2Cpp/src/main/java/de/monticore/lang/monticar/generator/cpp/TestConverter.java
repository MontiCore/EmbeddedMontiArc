/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.streamunits._ast.ASTComponentStreamUnits;
import de.monticore.lang.monticar.streamunits._ast.ASTNamedStreamUnits;
import de.monticore.lang.monticar.streamunits._ast.ASTPrecisionNumber;
import de.monticore.lang.monticar.streamunits._ast.ASTStreamInstruction;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;

/**
 */
public class TestConverter {

    public static FileContent generateMainTestFile(ComponentStreamUnitsSymbol symbol, EMAComponentInstanceSymbol instanceSymbol) {
        FileContent fileContent = new FileContent();
        fileContent.setFileName("main_" + GeneralHelperMethods.getTargetLanguageComponentName(symbol.getFullName()) + ".cpp");
        String fileContentString = "";
        fileContentString += getDefaultContent(symbol);
        ASTComponentStreamUnits ast = (ASTComponentStreamUnits) symbol.getAstNode().get();
        int executionCounter = 0;
        int executionAmount = ast.getNamedStreamUnitsList().get(0).getStream().getStreamInstructionList().size();
        while (executionCounter < executionAmount) {
            for (ASTNamedStreamUnits astNamedStreamUnit : ast.getNamedStreamUnitsList()) {
                fileContentString += getFileContentStringFor(instanceSymbol, astNamedStreamUnit, executionCounter);
            }
            fileContentString += "testInstance.execute();\n";
            for (ASTNamedStreamUnits astNamedStreamUnit : ast.getNamedStreamUnitsList()) {
                String portName = astNamedStreamUnit.getName();
                if (!instanceSymbol.getPortInstance(portName).get().isIncoming()) {
                    if (astNamedStreamUnit.getStream().getStreamInstructionList().size() > 0) {
                        ASTStreamInstruction streamInstruction = astNamedStreamUnit.getStream().getStreamInstructionList().get(executionCounter);
                        if (streamInstruction.getStreamValueOpt().isPresent() && streamInstruction.getStreamValueOpt().get().getPrecisionNumberOpt().isPresent()) {
                            ASTPrecisionNumber precisionNumber = streamInstruction.getStreamValueOpt().get().getPrecisionNumberOpt().get();
                            if (precisionNumber.getPrecisionOpt().isPresent()) {
                                fileContentString += "if(testInstance." + portName + ">" + "(" + MathConverter.getConvertedUnitNumber(precisionNumber.getNumberWithUnit()) + "-" + MathConverter.getConvertedUnitNumber(precisionNumber.getPrecisionOpt().get().getNumberWithUnit()) + ")";
                                fileContentString += "&& testInstance." + portName + "<" + "(" + MathConverter.getConvertedUnitNumber(precisionNumber.getNumberWithUnit()) + "+" + MathConverter.getConvertedUnitNumber(precisionNumber.getPrecisionOpt().get().getNumberWithUnit()) + ")";
                                fileContentString += "){";
                                fileContentString += "printf(\"Mismatch at executionStep " + executionCounter + "\");\n";
                                fileContentString += "octave_quit();\n";
                                fileContentString += "}\n";
                            } else {
                                fileContentString += "if(testInstance." + portName + "!=" + MathConverter.getConvertedUnitNumber(precisionNumber.getNumberWithUnit()) + "){";
                                fileContentString += "printf(\"Mismatch at executionStep " + executionCounter + "\");\n";
                                fileContentString += "octave_quit();\n";
                                fileContentString += "}\n";
                            }
                        }
                    }
                }
            }
            ++executionCounter;
        }
        fileContentString += "octave_quit();\n";
        fileContentString += "printf(\"Execution ended successfully!\\n\");\n";
        fileContentString += "}\n";

        fileContent.setFileContent(fileContentString);
        return fileContent;
    }

    public static String getDefaultContent(ComponentStreamUnitsSymbol symbol) {
        String fileContentString = "";
        String filePostfix = MathConverter.curBackend.getIncludeHeaderName().equals("armadillo") ? "" : ".h";
        fileContentString += "#ifndef M_PI\n" +
                "#define M_PI 3.14159265358979323846\n" +
                "#endif\n" + "#include \"" +  MathConverter.curBackend.getIncludeHeaderName() + filePostfix + "\"\n";

        if (MathConverter.curBackend.getBackendName().equals("OctaveBackend")) {
            fileContentString += "#include \"Helper.h\"\n";
        }
        ASTComponentStreamUnits ast = (ASTComponentStreamUnits) symbol.getAstNode().get();
        fileContentString += "#include \"" + GeneralHelperMethods.getTargetLanguageComponentName(symbol.getPackageName() + "." + Character.toLowerCase(ast.getComponentName().charAt(0)) + ast.getComponentName().substring(1)) + ".h\"\n";
        fileContentString += "int main(int argc, char** argv)\n" +
                "{\n";
        if (MathConverter.curBackend.getBackendName().equals("OctaveBackend"))
            fileContentString += "Helper::init();\n";
        fileContentString += "";
        fileContentString += GeneralHelperMethods.getTargetLanguageComponentName(symbol.getPackageName() + "." + Character.toLowerCase(ast.getComponentName().charAt(0)) + ast.getComponentName().substring(1));
        fileContentString += " testInstance;\n";
        fileContentString += "testInstance.init();\n";
        return fileContentString;
    }

    public static String getFileContentStringFor(EMAComponentInstanceSymbol instanceSymbol, ASTNamedStreamUnits astNamedStreamUnit, int executionCounter) {
        String fileContentString = "";
        String portName = astNamedStreamUnit.getName();
        if (instanceSymbol.getPortInstance(portName).get().isIncoming()) {
            if (astNamedStreamUnit.getStream().getStreamInstructionList().size() > 0) {
                ASTStreamInstruction streamInstruction = astNamedStreamUnit.getStream().getStreamInstructionList().get(executionCounter);
                if (streamInstruction.getStreamValueOpt().isPresent() && streamInstruction.getStreamValueOpt().get().getPrecisionNumberOpt().isPresent())
                    fileContentString += "testInstance." + portName + "=" + MathConverter.getConvertedUnitNumber(streamInstruction.
                            getStreamValueOpt().get().getPrecisionNumberOpt().get().getNumberWithUnit()) + ";";
            }
        }
        return fileContentString;
    }

}
