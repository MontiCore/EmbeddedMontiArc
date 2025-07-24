/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.instruction.ConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.monticore.lang.monticar.generator.cpp.viewmodel.LoggingViewModel;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.*;

/**
 */
public class LanguageUnitCPP extends LanguageUnit {

    List<String> includeStrings = new ArrayList<>();

    GeneratorCPP generatorCPP;

    public LanguageUnitCPP() {

    }

    public void setGeneratorCPP(GeneratorCPP generatorCPP) {
        this.generatorCPP = generatorCPP;
    }

    // add a function for seareching a string form list<String> in string
    private boolean containsString(String str, List<String> listOfStrings){
        boolean isContained = false;
        for(String foo: listOfStrings){
            if(str.toLowerCase().contains(foo.toLowerCase()))
                isContained = true;
        }
        return isContained;
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


                        EMAMBluePrint bluePrint = ComponentConverter.convertComponentSymbolToBluePrint((EMAComponentInstanceSymbol) symbol, (MathStatementsSymbol) nextSymbol, includeStrings, generatorCPP);
                        bluePrints.add(bluePrint);

                        ++i;
                    }

                } else {
                    EMAMBluePrint bluePrint = ComponentConverter.convertComponentSymbolToBluePrint((EMAComponentInstanceSymbol) symbol, includeStrings, generatorCPP);
                    bluePrints.add(bluePrint);
                }
            }

        }
    }

    public String getGeneratedHeader(TaggingResolver taggingResolver, EMAMBluePrintCPP bluePrint) {
//        ExecutionOrderFixer.fixExecutionOrder(taggingResolver, bluePrint, (GeneratorCPP) bluePrint.getGenerator());
        String resultString = "";
        //guard defines
        resultString += "#ifndef " + bluePrint.getName().toUpperCase() + "\n";
        resultString += "#define " + bluePrint.getName().toUpperCase() + "\n";
        if (generatorCPP.useMPIDefinitionFix())
            resultString += "#ifndef M_PI\n" +
                    "#define M_PI 3.14159265358979323846\n" +
                    "#endif\n";

        List<String> alreadyGeneratedIncludes = new ArrayList<>();
        List<String> alreadyGeneratedCVIncludes = new ArrayList<>();
        //includes
        //add default include
        String backendName = MathConverter.curBackend.getBackendName();
        if (backendName.equals("OctaveBackend")) {
            resultString += "#include \"octave/oct.h\"\n";
            alreadyGeneratedIncludes.add("octave/oct");
        } else if (backendName.equals("ArmadilloBackend")) {
            resultString += "#include \"" + MathConverter.curBackend.getIncludeHeaderName() + "\"\n";
            alreadyGeneratedIncludes.add(MathConverter.curBackend.getIncludeHeaderName());
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

        for (String string : bluePrint.getAdditionalUserIncludeStrings())
            resultString += String.format("#include \"%s.h\"\n", string);
        for (String string : bluePrint.getAdditionalStandardIncludeStrings())
            resultString += String.format("#include <%s>\n", string);
        for (String string : bluePrint.getAdditionalStandardIncludeStringsWithH())
            resultString += String.format("#include <%s.h>\n", string);
        for (String string : bluePrint.getAdditionalStandardIncludeStringsWithHPP())
            resultString += String.format("#include <%s.hpp>\n", string);

        if (generatorCPP.isExecutionLoggingActive)
            resultString += "#include <fstream>\n";

        for (String include : includeStrings)
            resultString += include;

        if (generatorCPP.useThreadingOptimizations()) {
            //if(MathConverter.curBackend.getBackendName().equals("OctaveBackend"))
            //resultString+="#include \"mingw.thread.h\"\n";
            //else if(MathConverter.curBackend.getBackendName().equals("ArmadilloBackend"))
            resultString += "#include <thread>\n";
        }
        if (MathConverter.curBackend.getBackendName().equals("ArmadilloBackend"))
            resultString += "using namespace arma;\n";

        for (String nameSpaceString : bluePrint.getAdditionalNameSpaceStrings())
            resultString += String.format("using namespace %s;\n", nameSpaceString);

        for (String typeDefString : bluePrint.getAdditionalTypeDefStrings())
            resultString += String.format("typedef %s;\n", typeDefString);

//        if(!bluePrint.cvIncludeStrings.isEmpty()){
//            resultString += "using namespace std;\n";
//        }

        //class definition start
        resultString += "class " + bluePrint.getName();
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

        if (bluePrint.getConstructor() != null)
            resultString += generateConstructor(bluePrint.getConstructor(), bluePrint);

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

    private String generateConstructor(Method constructor, EMAMBluePrintCPP bluePrint) {
        int counter = 0;
        String resultString = constructor.getName() + "(";
        for (Variable param : constructor.getParameters()) {
            if (counter == 0) {
                ++counter;
                resultString += param.getVariableType().getTypeNameTargetLanguage() + " " + param.getNameTargetLanguageFormat();
            } else {
                resultString += ", " + param.getVariableType().getTypeNameTargetLanguage() + " " + param.getNameTargetLanguageFormat();
            }
            if (param.isArray())
                resultString += "[" + param.getArraySize() + "]";
        }
        resultString += ") :\n";//TODO add semicolon when using source files

        for (Instruction instruction : constructor.getInstructions())
            resultString += instruction.getTargetLanguageInstruction();

        resultString += "{}\n";

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

    protected String generateMethod(Method method, EMAMBluePrint bluePrint){

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
            resultString += "logStart();\n";
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
            resultString += "logEnd();\n";
        }

        if (generatorCPP.isExecutionLoggingActive && method.getName().equals("init")) {
            resultString += "__EXECCOUNTER = 0;\n";
        }

        resultString += "}\n";

        if (generatorCPP.isExecutionLoggingActive && method.getName().equals("execute")) {
            resultString += AllTemplates.generateLogMethods(LoggingViewModel.fromBluePrint(bluePrint));
        }


        return resultString;
    }
}
