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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.instruction.ConnectInstructionCPP;
import de.monticore.lang.monticar.generator.order.ImplementExecutionOrder;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * @author Sascha Schneiders
 */
public class LanguageUnitCPP extends LanguageUnit {

    List<String> includeStrings = new ArrayList<>();

    GeneratorCPP generatorCPP;

    public LanguageUnitCPP() {

    }

    public void setGeneratorCPP(GeneratorCPP generatorCPP) {
        this.generatorCPP = generatorCPP;
    }

    public void generateBluePrints() {
        for (int i = 0; i < symbolsToConvert.size(); ++i) {
            Symbol symbol = symbolsToConvert.get(i);
            //only works with ExpandedComponentSymbols and MathStatementsSymbol

            if (symbol.isKindOf(EMAComponentInstanceSymbol.KIND)) {
                Log.info(symbol.toString(), "Current Symbol(size:" + symbolsToConvert.size() + ")" + ":");

                if (i + 1 < symbolsToConvert.size()) {
                    Symbol nextSymbol = symbolsToConvert.get(i + 1);
                    Log.info(nextSymbol.toString(), "Next Symbol:");
                    if (nextSymbol.isKindOf(MathStatementsSymbol.KIND)) {


                        BluePrint bluePrint = ComponentConverter.convertComponentSymbolToBluePrint((EMAComponentInstanceSymbol) symbol, (MathStatementsSymbol) nextSymbol, includeStrings, generatorCPP);
                        bluePrints.add(bluePrint);

                        ++i;
                    }

                } else {
                    BluePrint bluePrint = ComponentConverter.convertComponentSymbolToBluePrint((EMAComponentInstanceSymbol) symbol, includeStrings, generatorCPP);
                    bluePrints.add(bluePrint);
                }
            }

        }
    }

    public String getGeneratedHeader(TaggingResolver taggingResolver, BluePrintCPP bluePrint) {
        ExecutionOrderFixer.fixExecutionOrder(taggingResolver, bluePrint, (GeneratorCPP) bluePrint.getGenerator());
        String resultString = "";
        //guard defines
        resultString += "#ifndef " + bluePrint.getName().toUpperCase() + "\n";
        resultString += "#define " + bluePrint.getName().toUpperCase() + "\n";
        if (generatorCPP.useMPIDefinitionFix())
            resultString += "#ifndef M_PI\n" +
                    "#define M_PI 3.14159265358979323846\n" +
                    "#endif\n";

        List<String> alreadyGeneratedIncludes = new ArrayList<>();
        //includes
        //add default include
        String backendName = MathConverter.curBackend.getBackendName();
        if (backendName.equals("OctaveBackend") || backendName.equals("ArmadilloBackend")){
            String includeHeaderName = MathConverter.curBackend.getIncludeHeaderName();
            resultString += "#include \"" + includeHeaderName + ".h\"\n";
            alreadyGeneratedIncludes.add(includeHeaderName);
        }
        for (Variable v : bluePrint.getVariables()) {
            //TODO remove multiple same includes
            if (v.hasInclude()) {
                if (!alreadyGeneratedIncludes.contains(v.getVariableType().getIncludeName())) {
                    alreadyGeneratedIncludes.add(v.getVariableType().getIncludeName());
                    resultString += "#include \"" + v.getVariableType().getIncludeName() + ".h\"\n";
                }
            }
            if(v instanceof VariablePortValueChecker){
                if(!alreadyGeneratedIncludes.contains("PortValueCheck")){
                    alreadyGeneratedIncludes.add("PortValueCheck");
                    resultString += "#include \"PortValueCheck.h\"\n";
                }
            }

        }

        if (!alreadyGeneratedIncludes.contains("HelperA") && generatorCPP.isExecutionLoggingActive) {
            alreadyGeneratedIncludes.add("HelperA");
            resultString += "#include \"" + "HelperA" + ".h\"\n";
        }

        for (String string : bluePrint.getAdditionalIncludeStrings())
            resultString += "#include \"" + string + ".h\"\n";

        if (generatorCPP.isExecutionLoggingActive)
            resultString += "#include <fstream>\n";

        for (String include : includeStrings) {
            resultString += include;
        }
        if (generatorCPP.useThreadingOptimizations()) {
            //if(MathConverter.curBackend.getBackendName().equals("OctaveBackend"))
            //resultString+="#include \"mingw.thread.h\"\n";
            //else if(MathConverter.curBackend.getBackendName().equals("ArmadilloBackend"))
            resultString += "#include <thread>\n";
        }
        if (MathConverter.curBackend.getBackendName().equals("ArmadilloBackend")) {
            resultString += "using namespace arma;\n";
        }

        //class definition start
        resultString += "class " + bluePrint.getName() ;
        resultString += "{\n";

        //const variables
        for (String constString : bluePrint.getConsts())
            resultString += constString;

        //private variables
        for (Variable v : bluePrint.getVariables()) {
            if (v.isPublic()) {
                continue;
            }
            resultString += generateHeaderGenerateVariable(v);
        }

        //private methods
        for (Method method : bluePrint.getMethods()) {
            if(method.isPublic()){
                continue;
            }
            resultString += generateMethod(method, bluePrint);
        }

        resultString += "public:\n";
        if (generatorCPP.isExecutionLoggingActive) {
            resultString += "int __EXECCOUNTER;\n";
        }
        //input variable
        for (Variable v : bluePrint.getVariables()) {
            if (!v.isPublic()) {
                continue;
            }
            resultString += generateHeaderGenerateVariable(v);
        }

        //generate methods
        for (Method method : bluePrint.getMethods()) {
            if(!method.isPublic()){
                continue;
            }
            resultString += generateMethod(method, bluePrint);
        }


        resultString += "\n";
        //class definition end
        resultString += "};\n";

        //guard define end
        resultString += "#endif\n";
        Log.info(resultString, "Before RESSSS:");
        return resultString;
    }

    protected String generateHeaderGenerateVariable(Variable v){

        if(v instanceof VariablePortValueChecker){
            return ((VariablePortValueChecker)v).getTypeTargetLanguageFormat() +" "+ v.getNameTargetLanguageFormat()+";\n";
        }else {

            if (!v.isArray())
                return v.getVariableType().getTypeNameTargetLanguage() + " " + v.getNameTargetLanguageFormat() + ";\n";
            else
                return v.getVariableType().getTypeNameTargetLanguage() + " " + v.getNameTargetLanguageFormat() + "[" + v.getArraySize() + "]" + ";\n";
        }
    }

    protected String generateMethod(Method method, BluePrint bluePrint){

        int counter = 0;
        String resultString = method.getReturnTypeName() + " " + method.getName() + "(";
        for (Variable param : method.getParameters()) {
            if (counter == 0) {
                ++counter;
                resultString += param.getVariableType().getTypeNameTargetLanguage() + " " + param.getNameTargetLanguageFormat();
            } else {
                resultString += ", " + param.getVariableType().getTypeNameTargetLanguage() + " " + param.getNameTargetLanguageFormat();
            }
            if (param.isArray())
                resultString += "[" + param.getArraySize() + "]";
        }
        resultString += ")\n";//TODO add semicolon when using source files

        //method body start
        resultString += "{\n";

        if (generatorCPP.isExecutionLoggingActive && method.getName().equals("execute")) {
            resultString += "std::ofstream __LogExecutionFile;\n";
            resultString += "__LogExecutionFile.open(\"execution\" + std::to_string(__EXECCOUNTER) + \""+bluePrint.getOriginalSymbol().getPackageName()+"."+bluePrint.getOriginalSymbol().getName() +".res\");\n";

        }

        for (Instruction instruction : method.getInstructions()) {
            if (instruction instanceof ConnectInstructionCPP) {
                ConnectInstructionCPP connectInstructionCPP = (ConnectInstructionCPP) instruction;
                Log.info("v1: " + connectInstructionCPP.getVariable1().getName() + "v2: " + connectInstructionCPP.getVariable2().getName(), "Instruction:");
            } else if (instruction instanceof ExecuteInstruction) {
                ExecuteInstruction executeInstruction = (ExecuteInstruction) instruction;

            }
            Log.info(resultString, "beforRes:");
            resultString += instruction.getTargetLanguageInstruction();
            Log.info(resultString, "afterRes:");
        }

        if (generatorCPP.isExecutionLoggingActive && method.getName().equals("execute")) {
            for (Variable v : bluePrint.getVariables()) {
                if (v.hasAdditionalInformation(Variable.ORIGINPORT)) {
                    resultString += "__LogExecutionFile << \"" + v.getNameTargetLanguageFormat() + " : \";\n";
                    resultString += "toFileString(__LogExecutionFile, " + v.getNameTargetLanguageFormat() + ");\n";
                    resultString += "__LogExecutionFile << \"\\n\";\n";
                }
            }
        }
        if (generatorCPP.isExecutionLoggingActive && method.getName().equals("execute")) {
            resultString += "__LogExecutionFile.close();\n";
            resultString += "__EXECCOUNTER = __EXECCOUNTER + 1;\n";
        }
        if (generatorCPP.isExecutionLoggingActive && method.getName().equals("init")) {
            resultString += "__EXECCOUNTER = 0;\n";
        }

        //method body end
        resultString += "}\n";
        return resultString;
    }

}
