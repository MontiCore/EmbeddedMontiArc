/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.*;
import de.monticore.lang.monticar.generator.cpp.instruction.ConstantConnectInstructionCPP;
import de.monticore.lang.monticar.generator.optimization.MathInformationRegister;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Handles code generation for a component and its "subsymbols"
 */
public class ComponentConverter {

    public static BluePrintCPP currentBluePrint = null;
    public static List<String> namesOfFunctions = new ArrayList<>();
    public static List<MathCommand> usedMathCommand = new ArrayList<>();

    public static BluePrint convertComponentSymbolToBluePrint(EMAComponentInstanceSymbol componentSymbol, MathStatementsSymbol mathStatementsSymbol, List<String> includeStrings, GeneratorCPP generatorCPP) {
        BluePrintCPP bluePrint = new BluePrintCPP(GeneralHelperMethods.getTargetLanguageComponentName(componentSymbol.getFullName()));
        ComponentConverter.currentBluePrint = bluePrint;
        bluePrint.setGenerator(generatorCPP);
        bluePrint.setOriginalSymbol(componentSymbol);
        bluePrint.addDefineGenerics(componentSymbol);
        // Hint: the variables will be added here to the BluePrint
        addVariables(componentSymbol, bluePrint);
        // ToDo: you can fix the variables type here or later, so before it will be used
        BluePrintFixer.fixBluePrintDynamicVariableConnectRequestQueues(bluePrint);


        String lastNameWithoutArrayPart = "";
        for (EMAComponentInstanceSymbol instanceSymbol : componentSymbol.getSubComponents()) {
//            Unused: (?)
            int arrayBracketIndex = instanceSymbol.getName().indexOf("[");
            boolean generateComponentInstance = true;
            if (arrayBracketIndex != -1) {
                generateComponentInstance = !instanceSymbol.getName().substring(0, arrayBracketIndex).equals(lastNameWithoutArrayPart);
                lastNameWithoutArrayPart = instanceSymbol.getName().substring(0, arrayBracketIndex);
                Log.info(lastNameWithoutArrayPart, "Without:");
                Log.info(generateComponentInstance + "", "Bool:");
            }
            if (generateComponentInstance) {
            }
            bluePrint.addVariable(ComponentInstanceConverter.convertComponentInstanceSymbolToVariable(instanceSymbol, componentSymbol));
        }
        //create arrays from variables that only differ at the end by _number_
        BluePrintFixer.fixBluePrintVariableArrays(bluePrint);
        //ToDo: add bluePrintFixer.fixBluePrintCvVariableArrays;
        MathInformationFilter.filterStaticInformation(componentSymbol, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);
        //save function name
        if(mathStatementsSymbol != null) {
            List<MathExpressionSymbol> mathExpressionSymbols = mathStatementsSymbol.getMathExpressionSymbols();
            for(MathExpressionSymbol mathExpresionSymbol : mathExpressionSymbols){
                if (mathExpresionSymbol.isAssignmentExpression()) {
                    if (((MathAssignmentExpressionSymbol) mathExpresionSymbol).getExpressionSymbol() instanceof MathMatrixNameExpressionSymbol) {
                        namesOfFunctions.add(((MathMatrixNameExpressionSymbol) ((MathAssignmentExpressionSymbol) mathExpresionSymbol).getExpressionSymbol()).getNameToAccess());

                    }
                } else if(mathExpresionSymbol.isValueExpression()) {
                    if (((MathValueSymbol) mathExpresionSymbol).getValue() instanceof MathMatrixNameExpressionSymbol) {
                        boolean bool = namesOfFunctions.add(((MathMatrixNameExpressionSymbol) ((MathValueSymbol) mathExpresionSymbol).getValue()).getNameToAccess());
                    }
                }
            }
        }

        if(namesOfFunctions != null) {
            for(String nameOfFunction : namesOfFunctions){
                usedMathCommand.add(bluePrint.getMathCommandRegister().getMathCommand(nameOfFunction));
            }

        }
        //ToDo: add a BluePrintFixer.fixerBluePrintCVfuncitons(bluePrint, nameOfFunction);

        generateInitMethod(componentSymbol, bluePrint, generatorCPP, includeStrings);

        //generate execute method
        Method execute = ComponentConverterMethodGeneration.generateExecuteMethod(componentSymbol, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);


        EventConverter.generateEvents(execute, componentSymbol, bluePrint, mathStatementsSymbol,generatorCPP, includeStrings);

        extendInitMethod(componentSymbol, bluePrint, generatorCPP, includeStrings);


        bluePrint.addMethod(execute);

        EventConverter.generatePVCNextMethod(bluePrint);

        if(componentSymbol instanceof EMADynamicComponentInstanceSymbol){
            if(((EMADynamicComponentInstanceSymbol) componentSymbol).isDynamic()){
//                bluePrint.setHasSuperClass(Optional.of("__dynamicComponent"));
                //TODO: ADD parent dynamic function pointer
                addParentDynamicFunctionPointer(bluePrint);

                bluePrint.addAdditionalIncludeString("DynamicHelper");
            }
        }

        return bluePrint;
    }

    public static void addVariables(EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint) {
        //add parameters as variables
        for (EMAVariable variable : componentSymbol.getParameters()) {
            Log.debug("EMAVAR: " + variable.getName() + " " + variable.getType().toString(), "ComponentConverter");
            Variable var = new Variable();
            var.setName(variable.getName());
            var.setTypeNameMontiCar(variable.getType());
            bluePrint.addVariable(var);
            bluePrint.getMathInformationRegister().addVariable(var);
            var.setIsParameterVariable(true);
        }

        //add ports as variables to blueprint
        for (EMAPortInstanceSymbol port : componentSymbol.getPortInstanceList()) {
            //Config ports might already be added from adaptable Parameters
            if(!port.isConfig()) {
                bluePrint.addVariable(PortConverter.convertPortSymbolToVariable(port, port.getName(), bluePrint));
            }else{
                Set<String> paramNames = componentSymbol.getParameters().stream().map(EMAVariable::getName).collect(Collectors.toSet());
                if(!paramNames.contains(port.getName())){
                    //The port was not created by an adaptable parameter with the same name -> add
                    bluePrint.addVariable(PortConverter.convertPortSymbolToVariable(port, port.getName(), bluePrint));
                }
            }


        }
    }

    public static void addParentDynamicFunctionPointer(BluePrint bluePrint){
        if(!bluePrint.getVariable("(*__parent_dynamic)(void)").isPresent()) {
            Variable pd = new Variable();
            pd.setTypeNameTargetLanguage("void*");
            pd.setName("__parent");
            pd.setPublic(false);
            bluePrint.addVariable(pd);

            pd = new Variable();
            pd.setTypeNameTargetLanguage("void");
            pd.setName("(*__parent_dynamic)(void* pt2parrent, bool dynFunc, bool freeFunc)");
            pd.setPublic(false);
            bluePrint.addVariable(pd);

            if (bluePrint.getMethod("init").isPresent()) {
                bluePrint.getMethod("init").get().addInstruction(new TargetCodeInstruction("__parent = NULL;\n"));
                bluePrint.getMethod("init").get().addInstruction(new TargetCodeInstruction("__parent_dynamic = NULL;\n"));
            }

            if(!bluePrint.getMethod("set_Parent_Dynamic").isPresent()){
                Method m = new Method();
                m.setReturnTypeName("void");
                m.setName("set_Parent_Dynamic");

                pd = new Variable();
                pd.setTypeNameTargetLanguage("void* ");
                pd.setName("parentObj");
                m.addParameter(pd);

                pd = new Variable();
                pd.setTypeNameTargetLanguage("void");
                pd.setName("(*func)(void* pt2parrent, bool d, bool f)");
                m.addParameter(pd);

                m.addInstruction(new TargetCodeInstruction("__parent = parentObj;\n"));
                m.addInstruction(new TargetCodeInstruction("__parent_dynamic = func;\n"));
                bluePrint.addMethod(m);
            }
        }
    }

    public static Method generateInitMethod(EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings) {
        Method method = new Method("init", "void");
        bluePrint.addMethod(method);
        for (Variable v : bluePrint.getMathInformationRegister().getVariables()) {
            String oldName = v.getName();
            if (v.isArray()) {
                if (!v.getName().contains("[")) {
                    v.setName(v.getName() + "[1]");
                }
            }
            if (v.isStaticVariable()) {
                generateInitStaticVariablePart(method, v, bluePrint);
            } else {
                generateInitNonStaticVariable(method, v, bluePrint);
                //TODO: Handle the case for type changes like cube to vector<vector<Point>>
            }
            if (v.isArray())
                v.setName(oldName);
        }
        for (Variable v : bluePrint.getVariables()) {
            Log.info("Variable: " + v.getName(), "initBluePrintCreate:");

            if(v instanceof VariablePortValueChecker) {
//                ((VariablePortValueChecker) v).addInitInstructionsToMethod(method);
            }else if(v instanceof VariableConstantArray){
//                ((VariableConstantArray) v).generateInit(method);
            }else {
                if (v.isInputVariable() && !v.isConstantVariable()) {
                    //method.addParameter(v);
                    //Instruction instruction = new ConnectInstructionCPP(v, true, v, false);
                    //method.addInstruction(instruction);
                } else if (v.isConstantVariable()) {
                    Instruction instruction = new ConstantConnectInstructionCPP(v, v);
                    method.addInstruction(instruction);
                }
            }
        }

        for (EMAComponentInstanceSymbol subComponent : componentSymbol.getSubComponents()) {
            String parameterString = "";
            int i = 0;
            for (ASTExpression var : subComponent.getArguments()) {
                Log.debug(var.toString(), "ComponentConverter");
                if (i > 0)
                    parameterString += ", ";
                i++;
                parameterString += getExpressionParameterConversion(var);
            }
            String result = "";
            result += GeneralHelperMethods.getTargetLanguageVariableInstanceName(subComponent.getName()) + ".init(" + parameterString + ");\n";

            if((componentSymbol instanceof EMADynamicComponentInstanceSymbol) && (subComponent instanceof EMADynamicComponentInstanceSymbol)){
                if(((EMADynamicComponentInstanceSymbol) componentSymbol).isDynamic() && ((EMADynamicComponentInstanceSymbol) subComponent).isDynamic()){
                    result += GeneralHelperMethods.getTargetLanguageVariableInstanceName(subComponent.getName()) +".set_Parent_Dynamic(this, dynamicWrapper);\n";
                }
            }

            TargetCodeInstruction instruction = new TargetCodeInstruction(result);
            method.addInstruction(instruction);
        }
        return method;
    }

    public static void extendInitMethod(EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings){
        Optional<Method> init = bluePrint.getMethod("init");
        if(!init.isPresent()){
            init = Optional.of(new Method("init", "void"));
            bluePrint.addMethod(init.get());
        }
        for (Variable v : bluePrint.getVariables()) {
            Log.info("Variable: " + v.getName(), "initBluePrintExtension:");

            if(v instanceof VariablePortValueChecker) {
                ((VariablePortValueChecker) v).addInitInstructionsToMethod(init.get());
            }else if(v instanceof VariableConstantArray){
                ((VariableConstantArray) v).generateInit(init.get());
            }
        }
    }

    public static void generateInitStaticVariablePart(Method method, Variable v, BluePrintCPP bluePrint) {
        //TODO add static variable filter function before generate init method
        //extract the static variable and their possible assignments
        //generate the static variable and their assignment if present
        //check if value is constant matrix/vector definition
        //fix this too
        VariableStatic variableStatic = (VariableStatic) v;

        if (!variableStatic.getAssignmentSymbol().isPresent()) {
            String instructionString = "";
            if (variableStatic.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getMatrixTypeName())) {
                instructionString = MathConverter.getMatrixInitLine(v, bluePrint);
            } else if (variableStatic.getVariableType().getTypeNameTargetLanguage().equals("double")) {
                instructionString = MathInformationRegister.getVariableInitName(v, bluePrint) + "=0;\n";
            } else if (variableStatic.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getRowVectorTypeName())) {
                instructionString = MathConverter.getRowVectorInitLine(v, bluePrint);
            } else if (variableStatic.getVariableType().getTypeNameTargetLanguage().equals(MathConverter.curBackend.getColumnVectorTypeName())) {
                instructionString = MathConverter.getColumnVectorInitLine(v, bluePrint);
            }
            method.addInstruction(new TargetCodeInstruction(instructionString));
        } else {
            String instructionString = "";
            instructionString += variableStatic.getNameTargetLanguageFormat();
            instructionString += "=" + MathConverter.getConstantConversion(variableStatic.getAssignmentSymbol().get());
            instructionString += ";\n";
            method.addInstruction(new TargetCodeInstruction(instructionString));

        }
    }

    public static void generateInitNonStaticVariable(Method method, Variable v, BluePrintCPP bluePrint) {
        Log.info("v: " + v.getName(), "generateInitNonStaticVariable");
        if(v.isParameterVariable()){
            method.addInstruction(new TargetCodeInstruction("this->" + MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + MathInformationRegister.getVariableInitName(v, bluePrint) + ";\n"));
            method.addParameter(v);
        }else {
            Optional<String> initLine = MathConverter.getInitLine(v, bluePrint);
            initLine.ifPresent(s -> method.addInstruction(new TargetCodeInstruction(s)));
        }
    }

    public static String getExpressionParameterConversion(ASTExpression var) {
        String parameterString = "";
        if (var.getSymbolOpt().isPresent()) {
            boolean handled = false;
            MathExpressionSymbol symbol = (MathExpressionSymbol) var.getSymbolOpt().get();
            if (symbol.isMatrixExpression()) {
                MathMatrixExpressionSymbol mathMatrixExpressionSymbol = (MathMatrixExpressionSymbol) symbol;
                if (mathMatrixExpressionSymbol.isValueExpression()) {
                    parameterString = MathConverter.getConstantConversion((MathMatrixArithmeticValueSymbol) mathMatrixExpressionSymbol);
                    handled = true;

                }
            }
            if (!handled)
                parameterString += symbol.getTextualRepresentation();
        } else {
            parameterString += var.toString();
        }
        return parameterString;
    }

    public static BluePrint convertComponentSymbolToBluePrint(EMAComponentInstanceSymbol
                                                                      componentSymbol, List<String> includeStrings, GeneratorCPP generatorCPP) {
        return convertComponentSymbolToBluePrint(componentSymbol, null, includeStrings, generatorCPP);
    }
    public static void fixMathFunctions(MathExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP) {
        MathFunctionFixer.fixMathFunctions(mathExpressionSymbol, bluePrintCPP);
    }
}
