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
import de.monticore.lang.monticar.generator.cpp.loopSolver.CPPEquationSystemHelper;
import de.monticore.lang.monticar.generator.cpp.loopSolver.EquationSystemComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.loopSolver.RHSComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.optimization.MathInformationRegister;
import de.monticore.lang.monticar.semantics.executionOrder.ExecutionOrder;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystem;
import de.monticore.lang.monticar.semantics.loops.symbols.LoopComponentInstanceSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

/**
 *
 */
public class ComponentConverter {

    public static EMAMBluePrintCPP currentBluePrint = null;
    public static List<String> namesOfFunctions = new ArrayList<>();
    public static List<MathCommand> usedMathCommand = new ArrayList<>();
    public static HashMap<MathExpressionSymbol, MathExpressionProperties> tuples = new HashMap<>();

    public static EMAMBluePrint convertComponentSymbolToBluePrint(EMAComponentInstanceSymbol componentSymbol, MathStatementsSymbol mathStatementsSymbol, List<String> includeStrings, GeneratorCPP generatorCPP) {
        EMAMBluePrintCPP bluePrint = new EMAMBluePrintCPP(GeneralHelperMethods.getTargetLanguageComponentName(componentSymbol.getFullName()));
        ComponentConverter.currentBluePrint = bluePrint;
        bluePrint.setGenerator(generatorCPP);
        bluePrint.setOriginalSymbol(componentSymbol);
        bluePrint.addDefineGenerics(componentSymbol);
        addVariables(componentSymbol, bluePrint);

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
        if (componentSymbol instanceof LoopComponentInstanceSymbol) {
            EMAEquationSystem equationSystem = ((LoopComponentInstanceSymbol) componentSymbol).getEquationSystem();
            String eqsName = GeneralHelperMethods.getTargetLanguageComponentVariableInstanceName(equationSystem.getName());
            Variable eqs = new Variable("eqs", "");
            eqs.setVariableType(new VariableType("", eqsName, eqsName));
            eqs.addAdditionalInformation(Variable.COMPONENT);
            bluePrint.addVariable(eqs);
        }

        //create arrays from variables that only differ at the end by _number_
        BluePrintFixer.fixBluePrintVariableArrays(bluePrint);
        MathInformationFilter.filterStaticInformation(componentSymbol, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);
        //save functions names
        if (mathStatementsSymbol != null) {
            List<MathExpressionSymbol> mathExpressionSymbols = mathStatementsSymbol.getMathExpressionSymbols();
            for (MathExpressionSymbol mathExpressionSymbol : mathExpressionSymbols) {
                String nameOfFunction = getNameOfMathCommand(mathExpressionSymbol);
                namesOfFunctions.add(nameOfFunction);
                fixFunctionTypes(nameOfFunction, mathExpressionSymbol, bluePrint);
            }

            if (mathExpressionSymbols.size() >= 1) {
                for (MathExpressionSymbol mathExpressionSymbol : mathExpressionSymbols) {
                    MathExpressionProperties properties = new MathExpressionProperties();
                    MathConverter.setPropertiesForMathExpression(mathExpressionSymbols, mathExpressionSymbol, bluePrint, properties);
                    if (mathExpressionSymbol.isAssignmentExpression() && ((MathAssignmentExpressionSymbol) mathExpressionSymbol).getExpressionSymbol().isMatrixExpression()) {
                        MathMatrixExpressionSymbol mathMatrixExpressionSymbol = (MathMatrixExpressionSymbol) ((MathAssignmentExpressionSymbol) mathExpressionSymbol).getExpressionSymbol();
                        if (mathMatrixExpressionSymbol.isMatrixNameExpression()) {
                            MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) ((MathAssignmentExpressionSymbol) mathExpressionSymbol).getExpressionSymbol();
                            ComponentConverter.tuples.put(mathMatrixNameExpressionSymbol, properties);
                        }
                    } else {
                        ComponentConverter.tuples.put(mathExpressionSymbol, properties);
                    }
                }
            }
            redefineVariables(mathExpressionSymbols, bluePrint);
        }

        if (namesOfFunctions != null) {
            for (String nameOfFunction : namesOfFunctions) {
                usedMathCommand.add(bluePrint.getMathCommandRegister().getMathCommand(nameOfFunction));
            }
        }


        bluePrint.addMethod(new Method("init", "void"));
        bluePrint.addMethod(new Method(ComponentConverterMethodGeneration.EXECUTE_METHOD_NAME, "void"));

        if (mathStatementsSymbol != null) {
            for (MathExpressionSymbol mathExpressionSymbol : mathStatementsSymbol.getMathExpressionSymbols()) {
                MathFunctionFixer.fixMathFunctions(mathExpressionSymbol, bluePrint);
            }
        }

        generateInitMethod(componentSymbol, bluePrint, generatorCPP, includeStrings);

        //generate execute method
        Method execute = ComponentConverterMethodGeneration.generateExecuteMethod(componentSymbol, bluePrint,
                mathStatementsSymbol, generatorCPP, includeStrings);

        EventConverter.generateEvents(execute, componentSymbol, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);

        extendInitMethod(componentSymbol, bluePrint, generatorCPP, includeStrings);

        EventConverter.generatePVCNextMethod(bluePrint);

        if (componentSymbol instanceof EMADynamicComponentInstanceSymbol) {
            if (((EMADynamicComponentInstanceSymbol) componentSymbol).isDynamic()) {
//                bluePrint.setHasSuperClass(Optional.of("__dynamicComponent"));
                //TODO: ADD parent dynamic function pointer
                addParentDynamicFunctionPointer(bluePrint);

                bluePrint.addAdditionalUserIncludeStrings("DynamicHelper");
            }
        }
        if (generatorCPP.isGenerateCV) {
            MathCommand.redefineArmaMat(bluePrint);
            MathCommand.redefineInit(bluePrint);
        }
        if (componentSymbol instanceof EquationSystemComponentInstanceSymbol) {
            bluePrint.getMethod("init").get().addInstruction(
                    new TargetCodeInstruction("rhs.init();\n"));
        }

        // generate output and update
        if (ExecutionOrder.needsOutputAndUpdateSplit(componentSymbol)) {
            Method output = ComponentConverterMethodGeneration.generateOutputMethod(componentSymbol, bluePrint,
                    mathStatementsSymbol, generatorCPP, includeStrings);

            Method update = ComponentConverterMethodGeneration.generateUpdateMethod(componentSymbol, bluePrint,
                    mathStatementsSymbol, generatorCPP, includeStrings);

            bluePrint.addMethod(output);
            bluePrint.addMethod(update);
        }

        addOthersToExecuteMethod(bluePrint);

        return bluePrint;
    }

    private static void addOthersToExecuteMethod(EMAMBluePrintCPP bluePrint) {
        Optional<Method> executeOpt = bluePrint.getMethod("execute");
        if (executeOpt.isPresent()) {
            Method exMethod = executeOpt.get();
            if (bluePrint.getMethod("next").isPresent()) {
                exMethod.addInstruction(0, new TargetCodeInstruction("next();\n"));
            }

            if (bluePrint.getMethod("dynamic").isPresent()) {
                exMethod.addInstruction(0, new TargetCodeInstruction("dynamic();\n"));
            }
        }
    }

    public static void addVariables(EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint) {
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
            String name = (componentSymbol instanceof EquationSystemComponentInstanceSymbol ||
                    componentSymbol instanceof RHSComponentInstanceSymbol) ?
                    CPPEquationSystemHelper.getNameOfPort(port) :
                    port.getName();
            if (!port.isConfig()) {
                bluePrint.addVariable(PortConverter.convertPortSymbolToVariable(port, name, bluePrint));
            } else {
                Set<String> paramNames = componentSymbol.getParameters().stream().map(EMAVariable::getName).collect(Collectors.toSet());
                if (!paramNames.contains(port.getName())) {
                    //The port was not created by an adaptable parameter with the same name -> add
                    bluePrint.addVariable(PortConverter.convertPortSymbolToVariable(port, name, bluePrint));
                }
            }


        }
    }

    public static void addParentDynamicFunctionPointer(EMAMBluePrint bluePrint) {
        if (!bluePrint.getVariable("(*__parent_dynamic)(void)").isPresent()) {
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

            if (!bluePrint.getMethod("set_Parent_Dynamic").isPresent()) {
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

    public static Method generateInitMethod(EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings) {
        Method method = bluePrint.getMethod("init").orElse(new Method("init", "void"));
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
            } else if (v.getAdditionalInformation().contains(Variable.COMPONENT)
                    || v.isInputVariable()
                    || v.isParameterVariable()
                    || v.getAdditionalInformation().contains(Variable.OUTGOING)
                    || v.getAdditionalInformation().contains(Variable.ORIGINPORT)) {
                generateInitNonStaticVariable(method, v, bluePrint);
            }
            if (v.isArray())
                v.setName(oldName);
        }
        for (Variable v : bluePrint.getVariables()) {
            Log.info("Variable: " + v.getName(), "initBluePrintCreate:");

            if (v instanceof VariablePortValueChecker) {
//                ((VariablePortValueChecker) v).addInitInstructionsToMethod(method);
            } else if (v instanceof VariableConstantArray) {
//                ((VariableConstantArray) v).generateInit(method);
            } else {
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

            String subComponentName = subComponent.getName();
            if (componentSymbol instanceof RHSComponentInstanceSymbol)
                subComponentName = GeneralHelperMethods.getTargetLanguageComponentName(subComponent.getFullName());
            result += GeneralHelperMethods.getTargetLanguageVariableInstanceName(subComponentName) + ".init(" + parameterString + ");\n";

            if ((componentSymbol instanceof EMADynamicComponentInstanceSymbol) && (subComponent instanceof EMADynamicComponentInstanceSymbol)) {
                if (((EMADynamicComponentInstanceSymbol) componentSymbol).isDynamic() && ((EMADynamicComponentInstanceSymbol) subComponent).isDynamic()) {
                    result += GeneralHelperMethods.getTargetLanguageVariableInstanceName(subComponent.getName()) + ".set_Parent_Dynamic(this, dynamicWrapper);\n";
                }
            }

            TargetCodeInstruction instruction = new TargetCodeInstruction(result);
            method.addInstruction(instruction);
        }
        return method;
    }

    public static void extendInitMethod(EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings) {
        Optional<Method> init = bluePrint.getMethod("init");
        if (!init.isPresent()) {
            init = Optional.of(new Method("init", "void"));
            bluePrint.addMethod(init.get());
        }
        for (Variable v : bluePrint.getVariables()) {
            Log.info("Variable: " + v.getName(), "initBluePrintExtension:");

            if (v instanceof VariablePortValueChecker) {
                ((VariablePortValueChecker) v).addInitInstructionsToMethod(init.get());
            } else if (v instanceof VariableConstantArray) {
                ((VariableConstantArray) v).generateInit(init.get());
            }
        }
    }

    public static void generateInitStaticVariablePart(Method method, Variable v, EMAMBluePrintCPP bluePrint) {
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

    public static void generateInitNonStaticVariable(Method method, Variable v, EMAMBluePrintCPP bluePrint) {
        Log.info("v: " + v.getName(), "generateInitNonStaticVariable");
        if (v.isParameterVariable()) {
            method.addInstruction(new TargetCodeInstruction("this->" + MathInformationRegister.getVariableInitName(v, bluePrint) + "=" + MathInformationRegister.getVariableInitName(v, bluePrint) + ";\n"));
            method.addParameter(v);
        } else {
            Optional<String> initLine = MathConverter.getInitLine(v, bluePrint);
            initLine.ifPresent(s -> method.addInstruction(new TargetCodeInstruction(s)));
        }
    }

    public static String getNameOfMathCommand(MathExpressionSymbol mathExpressionSymbol) {
        String nameOfFunction = "";
        if (mathExpressionSymbol.isAssignmentExpression()) {
            if (((MathAssignmentExpressionSymbol) mathExpressionSymbol).getExpressionSymbol() instanceof MathMatrixNameExpressionSymbol) {
                nameOfFunction = ((MathMatrixNameExpressionSymbol) ((MathAssignmentExpressionSymbol) mathExpressionSymbol).getExpressionSymbol()).getNameToAccess();
            }
        } else if (mathExpressionSymbol.isValueExpression()) {
            if (((MathValueSymbol) mathExpressionSymbol).getValue() instanceof MathMatrixNameExpressionSymbol) {
                nameOfFunction = ((MathMatrixNameExpressionSymbol) ((MathValueSymbol) mathExpressionSymbol).getValue()).getNameToAccess();
            }
        }
        return nameOfFunction;
    }

    public static String getNameOfOutput(MathExpressionSymbol mathExpressionSymbol) {
        String outputName = "";
        if (mathExpressionSymbol.isAssignmentExpression()) {
            if (((MathAssignmentExpressionSymbol) mathExpressionSymbol).getExpressionSymbol() instanceof MathMatrixNameExpressionSymbol) {
                outputName = ((MathAssignmentExpressionSymbol) mathExpressionSymbol).getNameOfMathValue();

            }
        } else if (mathExpressionSymbol.isValueExpression()) {
            if (((MathValueSymbol) mathExpressionSymbol).getValue() instanceof MathMatrixNameExpressionSymbol) {
                outputName = ((MathValueSymbol) mathExpressionSymbol).getName();
            }
        }
        return outputName;
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

    public static EMAMBluePrint convertComponentSymbolToBluePrint(EMAComponentInstanceSymbol
                                                                          componentSymbol, List<String> includeStrings, GeneratorCPP generatorCPP) {
        return convertComponentSymbolToBluePrint(componentSymbol, null, includeStrings, generatorCPP);
    }

    public static void fixMathFunctions(MathExpressionSymbol mathExpressionSymbol, EMAMBluePrintCPP bluePrintCPP) {
        MathFunctionFixer.fixMathFunctions(mathExpressionSymbol, bluePrintCPP);
    }

    private static void fixFunctionTypes(String nameOfFunction, MathExpressionSymbol mathExpressionSymbol, EMAMBluePrintCPP bluePrintCPP) {
        String outputName = getNameOfOutput(mathExpressionSymbol);
        switch (nameOfFunction) {
            case "findContours":
                fixVariableType(outputName, bluePrintCPP, "Q", "std::vector<std::vector<cv::Point>>", "");
                break;
            case "largestContour":
                fixVariableType(outputName, bluePrintCPP, "Q", "std::vector<cv::Point>", "");
                break;
            case "boundingRect":
                fixVariableType(outputName, bluePrintCPP, "Q", "cv::Rect", "");
                break;
        }
    }

    public static void redefineVariables(List<MathExpressionSymbol> mathExpressionSymbols, EMAMBluePrintCPP bluePrintCPP) {
        for (MathExpressionSymbol mathExpressionSymbol : mathExpressionSymbols) {
            if (mathExpressionSymbol.isAssignmentExpression() && ((MathAssignmentExpressionSymbol) mathExpressionSymbol).getExpressionSymbol().isMatrixExpression()) {
                MathMatrixExpressionSymbol mathMatrixExpressionSymbol = (MathMatrixExpressionSymbol) ((MathAssignmentExpressionSymbol) mathExpressionSymbol).getExpressionSymbol();
                if (mathMatrixExpressionSymbol.isMatrixNameExpression()) {
                    MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) ((MathAssignmentExpressionSymbol) mathExpressionSymbol).getExpressionSymbol();
                    String nameOfFirstParameter = mathMatrixNameExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(0).getTextualRepresentation();
                    String nameOfOutput = getNameOfOutput(mathExpressionSymbol);
                    MathExpressionProperties properties = tuples.get(mathMatrixNameExpressionSymbol);
                    if (properties != null) {
                        if (properties.isPreCV()) {
                            fixVariableType(nameOfFirstParameter, bluePrintCPP, "CommonMatrixType", "cv::Mat", "");
                        }
                        if (properties.isSucCV()) {
                            fixVariableType(nameOfOutput, bluePrintCPP, "CommonMatrixType", "cv::Mat", "");
                        }
                    }
                }
            }
        }
    }


    public static void fixVariableType(String variableName, EMAMBluePrintCPP bluePrint, String typeNameMontiCar, String typeNameTargetLanguage, String includeName) {
        List<Variable> variables = bluePrint.getVariables();
        for (Variable var : variables) {
            if (var.getName().equals(variableName)) {
                fixType(var, typeNameMontiCar, typeNameTargetLanguage, includeName);
            }
        }
    }

    private static void fixType(Variable variable, String typeNameMontiCar, String typeNameTargetLanguage, String includeName) {
        VariableType varTypeNew = new VariableType(typeNameMontiCar, typeNameTargetLanguage, includeName);
        variable.setVariableType(varTypeNew);
    }
}
