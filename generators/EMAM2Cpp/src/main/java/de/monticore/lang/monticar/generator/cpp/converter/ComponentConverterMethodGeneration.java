/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.instruction.ConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.instruction.ExecuteDynamicConnects;
import de.monticore.lang.monticar.generator.cpp.loopSolver.CPPEquationSystemHelper;
import de.monticore.lang.monticar.generator.cpp.loopSolver.EquationSystemComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.loopSolver.RHSComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.MathStringExpression;
import de.monticore.lang.monticar.generator.optimization.MathOptimizer;
import de.monticore.lang.monticar.semantics.executionOrder.SList;
import de.monticore.lang.monticar.semantics.executionOrder.SListEntry;
import de.monticore.lang.monticar.semantics.helper.Find;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.analyze.GenerateComponentFunction;
import de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper;
import de.monticore.lang.monticar.semantics.loops.graph.EMAAtomicConnectorInstance;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystem;
import de.monticore.lang.monticar.semantics.loops.symbols.LoopComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.*;
import de.se_rwth.commons.logging.Log;

import java.util.*;

import static de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter.getNameOfOutput;

/**
 *
 */
public class ComponentConverterMethodGeneration {
    public static final String EXECUTE_METHOD_NAME = "execute";
    public static final String OUTPUT_METHOD_NAME = "output";
    public static final String UPDATE_METHOD_NAME = "update";
    public static int currentGenerationIndex = 0;
    public static EMAComponentInstanceSymbol currentComponentSymbol = null;


    public static Method generateExecuteMethod(EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint,
                                               MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP,
                                               List<String> includeStrings) {


        currentComponentSymbol = componentSymbol;
/*
        if(componentSymbol instanceof EMADynamicComponentInstanceSymbol){
            EMADynamicComponentInstanceSymbol dynComp = (EMADynamicComponentInstanceSymbol)componentSymbol;
            if(dynComp.getEventHandlers().size() > 0) {
                Method execute = new Method("execute", "void");
                Method inner = new Method("execute_inner", "void");
                inner.setPublic(false);
                generateExecuteMethodInner(inner, componentSymbol, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);

                EventHandlerMethodsGenerator.generateInputsOfEventHandlers(componentSymbol, bluePrint,execute);

                if (!inner.getInstructions().isEmpty()){
                    execute.addInstruction(new TargetCodeInstruction("//--------------------\n"));
                    execute.addInstruction(new TargetCodeInstruction("execute_inner();\n"));
                    execute.addInstruction(new TargetCodeInstruction("//--------------------\n"));
                    bluePrint.addMethod(inner);
                }

                EventHandlerMethodsGenerator.generateOutputsOfEventHandlers(componentSymbol, bluePrint,execute);

                bluePrint.addMethod(execute);
                return;
            }
        }
*/
        Method exMethod = bluePrint.getMethod(EXECUTE_METHOD_NAME).orElse(new Method(EXECUTE_METHOD_NAME, "void"));
        bluePrint.addMethod(exMethod);

        if (componentSymbol instanceof LoopComponentInstanceSymbol) {
            generateExecuteForLoopComponent((LoopComponentInstanceSymbol) componentSymbol, exMethod, bluePrint);
        } else if (componentSymbol instanceof EquationSystemComponentInstanceSymbol) {
            generateExecuteExecuteForEquationSystemComponent((EquationSystemComponentInstanceSymbol) componentSymbol, bluePrint, generatorCPP, includeStrings, exMethod);
        } else if (componentSymbol instanceof RHSComponentInstanceSymbol) {
            generateExecuteForRHSComponent((RHSComponentInstanceSymbol) componentSymbol, bluePrint, generatorCPP, includeStrings, exMethod);
        } else
            generateExecuteMethodInner(exMethod, componentSymbol, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);

        // Add other instructions
        fixExecuteDynamicConnects(exMethod);

        // rearrange execute to be last
        bluePrint.getMethods().remove(exMethod);
        bluePrint.addMethod(exMethod);

        return exMethod;
    }

    private static void fixExecuteDynamicConnects(Method exMethod) {
        List<Instruction> instructions = exMethod.getInstructions();
        List<Integer> idx = new ArrayList<>();

        for(int i = 1; i < instructions.size(); ++i){
            if((instructions.get(i-1) instanceof ExecuteDynamicConnects) && (instructions.get(i) instanceof ExecuteDynamicConnects)){
                ExecuteDynamicConnects a = (ExecuteDynamicConnects) instructions.get(i-1);
                ExecuteDynamicConnects b = (ExecuteDynamicConnects) instructions.get(i);
                if(a.getBeforeComponentName().equals(b.getBeforeComponentName())){
                    idx.add(i);
                }
            }
        }

        Collections.sort(idx, Collections.reverseOrder());
        for(int i : idx){
            instructions.remove(i);
        }
        exMethod.setInstructions(instructions);
    }

    private static void generateExecuteForRHSComponent(RHSComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings, Method exMethod) {
        SemiExplicitForm semiExplicitForm = componentSymbol.getSemiExplicitForm();
        addRHSOutputCalls(componentSymbol, exMethod, bluePrint, generatorCPP);
        CPPEquationSystemHelper.handleRHS(semiExplicitForm, bluePrint, generatorCPP, includeStrings);
        addRHSUpdateCalls(componentSymbol, exMethod, bluePrint, generatorCPP);
    }

    private static void generateExecuteExecuteForEquationSystemComponent(EquationSystemComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings, Method exMethod) {
        for (Variable variable : bluePrint.getVariables())
            if (variable.getAdditionalInformation().contains("Incoming"))
                exMethod.addInstruction(new TargetCodeInstruction(String.format("rhs.%s = %s;\n", variable.getName(), variable.getName())));
        SemiExplicitForm semiExplicitForm = componentSymbol.getSemiExplicitForm();
        CPPEquationSystemHelper.handleEquationSystem(semiExplicitForm, bluePrint, generatorCPP, includeStrings);
    }

    private static void addRHSOutputCalls(RHSComponentInstanceSymbol componentSymbol, Method exMethod, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP) {
        List<Instruction> newInstructions = new LinkedList<>();
        for (EMAComponentInstanceSymbol subComponent : componentSymbol.getSubComponents()) {
            String name = GeneralHelperMethods.getTargetLanguageComponentVariableInstanceName(subComponent.getFullName());
            newInstructions.add(new OutputInstruction(name, bluePrint, generatorCPP.useThreadingOptimizations()));
        }
        exMethod.addInstructions(newInstructions);
        if (generatorCPP.useThreadingOptimizations())
            for (EMAComponentInstanceSymbol subComponent : componentSymbol.getSubComponents()) {
                String name = GeneralHelperMethods.getTargetLanguageComponentVariableInstanceName(subComponent.getFullName());
                for (Instruction instruction : newInstructions) {
                    if (instruction.isExecuteInstruction()) {
                        ExecuteInstruction executeInstruction = (ExecuteInstruction) instruction;
                        exMethod.addInstruction(new TargetCodeInstruction(executeInstruction.getThreadName() + ".join();\n"));
                    }
                }
            }
    }

    private static void addRHSUpdateCalls(RHSComponentInstanceSymbol componentSymbol, Method exMethod, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP) {
        List<Instruction> newInstructions = new LinkedList<>();
        for (EMAComponentInstanceSymbol subComponent : componentSymbol.getSubComponents()) {
            String name = GeneralHelperMethods.getTargetLanguageComponentVariableInstanceName(subComponent.getFullName());
            newInstructions.add(new UpdateInstruction(name, bluePrint, generatorCPP.useThreadingOptimizations()));
        }
        exMethod.addInstructions(newInstructions);
    }

    protected static void generateExecuteMethodInner(Method method, EMAComponentInstanceSymbol componentSymbol,
                                                     EMAMBluePrintCPP bluePrint,
                                                     MathStatementsSymbol mathStatementsSymbol,
                                                     GeneratorCPP generatorCPP, List<String> includeStrings) {
        if (mathStatementsSymbol == null) {
//            Collection<EMAConnectorInstanceSymbol> connectors = new HashSet<>();
//            connectors.addAll(Find.allAtomicConnectors(componentSymbol));
//            generateConnectors(connectors, bluePrint, method);
            if (componentSymbol.isNonVirtual() || !componentSymbol.getParent().isPresent())
                generateSListConnectors(Find.allAtomicConnectors(componentSymbol), bluePrint, method);
        } else
            handleMathStatementGeneration(method, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);
    }

    private static void generateExecuteForLoopComponent(LoopComponentInstanceSymbol componentSymbol, Method method,
                                                        EMAMBluePrintCPP bluePrintCPP) {
        EMAEquationSystem equationSystem = componentSymbol.getEquationSystem();
        for (EMAPortInstanceSymbol inport : equationSystem.getIncomingPorts()) {
            Optional<EMAPortInstanceSymbol> originalSourcePort = equationSystem.getAtomicSourceOf(inport);
            Optional<EMAPortInstanceSymbol> currentPort = componentSymbol.getIncomingPortInstances().stream()
                    .filter(i -> ConnectionHelper.sourceOf(i).equals(originalSourcePort))
                    .findFirst();
            if (currentPort.isPresent()) {
                String sourceName = currentPort.get().getName();
                String targetName = String.join(".", "eqs",
                        CPPEquationSystemHelper.getNameOfPort(inport));
                Variable v1 = PortConverter.convertPortSymbolToVariable(currentPort.get(), sourceName, bluePrintCPP);
                Variable v2 = PortConverter.convertPortSymbolToVariable(inport, targetName, bluePrintCPP);
                method.addInstruction(new ConnectInstructionCPP(v2, v1));
            } else Log.error("Missing information for equation system TODO");
        }
        method.addInstruction(new ExecuteInstruction("eqs", bluePrintCPP, false));
        for (EMAPortInstanceSymbol outport : componentSymbol.getOutgoingPortInstances()) {
            Optional<EMAPortInstanceSymbol> eqsVar = equationSystem.getOutgoingPorts().stream()
                    .filter(p -> p.getFullName().equals(outport.getFullName()))
                    .findFirst();
            if (eqsVar.isPresent()) {
                String sourceName = String.join(".", "eqs",
                        CPPEquationSystemHelper.getNameOfPort(eqsVar.get()));
                String targetName = outport.getName();
                Variable v1 = PortConverter.convertPortSymbolToVariable(eqsVar.get(), sourceName, bluePrintCPP);
                Variable v2 = PortConverter.convertPortSymbolToVariable(outport, targetName, bluePrintCPP);
                method.addInstruction(new ConnectInstructionCPP(v2, v1));
            } else Log.error("Could not find corresponding port");
        }
    }

    public static void generateConnectors(Collection<EMAConnectorInstanceSymbol> connectors, EMAMBluePrintCPP bluePrint, Method method) {
        for (EMAConnectorInstanceSymbol connector : connectors) {
            if (!connector.isConstant()) {
                Log.info("source:" + connector.getSource() + " target:" + connector.getTarget(), "Port info:");
                Variable v1 = PortConverter.getVariableForPortSymbol(connector, connector.getSource(), bluePrint);
                Variable v2 = PortConverter.getVariableForPortSymbol(connector, connector.getTarget(), bluePrint);
                if (!connector.getComponentInstance().getIncomingPortInstances().contains(connector.getSourcePort()))
                    v1.addAdditionalInformation(Variable.CROSSCOMPONENT);
                if (!connector.getComponentInstance().getOutgoingPortInstances().contains(connector.getTargetPort()))
                    v2.addAdditionalInformation(Variable.CROSSCOMPONENT);

                Log.info("v1: " + v1.getName() + " v2: " + v2.getName(), "Variable Info:");
                Log.info("v1: " + v1.getNameTargetLanguageFormat() + " v2: " + v2.getNameTargetLanguageFormat(), "Variable Info:");

                Instruction instruction = new ConnectInstructionCPP(v2, v1);
                method.addInstruction(instruction);
            } else {
                if (connector.getSourcePort().isConstant()) {
                    EMAPortInstanceSymbol constPort = connector.getSourcePort();
                    Variable v1 = new Variable();
                    v1.setName(constPort.getConstantValue().get().getValueAsString());
                    Variable v2 = PortConverter.getVariableForPortSymbol(connector, connector.getTarget(), bluePrint);


                    Instruction instruction = new ConnectInstructionCPP(v2, v1);
                    method.addInstruction(instruction);
                } else if (connector.getTargetPort().isConstant()) {
                    EMAPortInstanceSymbol constPort = connector.getTargetPort();
                    Variable v2 = new Variable();
                    v2.setName(constPort.getConstantValue().get().getValueAsString());
                    Variable v1 = PortConverter.getVariableForPortSymbol(connector, connector.getSource(), bluePrint);


                    Instruction instruction = new ConnectInstructionCPP(v2, v1);
                    method.addInstruction(instruction);
                } else {
                    Log.error("0xWRONGCONNECTOR the connector is constant but target nor source are constant");
                }
            }
        }
    }

    public static void generateSListConnectors(Collection<EMAAtomicConnectorInstance> connectors,
                                               EMAMBluePrintCPP bluePrint,
                                               Method method) {
        for (EMAPortInstanceSymbol inport : currentComponentSymbol.getIncomingPortInstances())
            if (!inport.isConstant())
                method.addInstructions(createTargetConnectors(bluePrint, inport));

        generateConstantConnectors(connectors, bluePrint, method);

        boolean useThreadingOptimizations = false;
        if (bluePrint.getGenerator() instanceof GeneratorCPP
                && ((GeneratorCPP) bluePrint.getGenerator()).useThreadingOptimizations())
            useThreadingOptimizations = true;
        if (!useThreadingOptimizations)
            generateComponentExecutionForNonThreadedSList(bluePrint, method);
        else
            generateComponentExecutionForThreadedSList(bluePrint, method);
    }

    private static void generateComponentExecutionForNonThreadedSList(EMAMBluePrintCPP bluePrint, Method method) {
        List<SListEntry> slist = SList.sListSerial(currentComponentSymbol);
        generateComponentExecutionForSList(bluePrint, method, slist, false);
    }

    private static void generateComponentExecutionForThreadedSList(EMAMBluePrintCPP bluePrint, Method method) {
        List<List<SListEntry>> slist = SList.sListParallel(currentComponentSymbol);
        int lastIndex = 0;
        for (List<SListEntry> sListEntries : slist) {
            generateComponentExecutionForSList(bluePrint, method, sListEntries, true);
        }
    }

    private static void generateComponentExecutionForSList(EMAMBluePrintCPP bluePrint, Method method,
                                                           List<SListEntry> sListEntries, boolean allCanBeThreaded) {
        List<Instruction> newInstructions = new LinkedList<>();
        for (SListEntry entry : sListEntries) {
            entry.getComponent().getFullName();
            String componentName = NameHelper.calculatePartialName(entry.getComponent(), currentComponentSymbol);
            if (entry.isExecuteCall())
                newInstructions.add(new ExecuteInstruction(componentName, bluePrint, allCanBeThreaded));
            else if (entry.isOutputCall())
                newInstructions.add(new OutputInstruction(componentName, bluePrint, allCanBeThreaded));
            else if (entry.isUpdateCall())
                newInstructions.add(new UpdateInstruction(componentName, bluePrint, allCanBeThreaded));

            // if allCanBeThreaded delay after all are calculated
            if (!allCanBeThreaded && (entry.isExecuteCall() || entry.isOutputCall())) {
                for (EMAPortInstanceSymbol outport : entry.getComponent().getOutgoingPortInstances())
                    newInstructions.addAll(createTargetConnectors(bluePrint, outport));
            }
        }

        if (allCanBeThreaded) {
            List<Instruction> joinInstructions = new LinkedList<>();
            for (Instruction instruction : newInstructions) {
                if (instruction.isExecuteInstruction()) {
                    ExecuteInstruction executeInstruction = (ExecuteInstruction) instruction;
                    joinInstructions.add(new TargetCodeInstruction(executeInstruction.getThreadName() + ".join();\n"));
                }
            }
            newInstructions.addAll(joinInstructions);
            for (SListEntry entry : sListEntries) {
                if (entry.isExecuteCall() || entry.isOutputCall()) {
                    for (EMAPortInstanceSymbol outport : entry.getComponent().getOutgoingPortInstances())
                        newInstructions.addAll(createTargetConnectors(bluePrint, outport));
                }
            }
        }
        method.addInstructions(newInstructions);
    }

    private static void generateConstantConnectors(Collection<EMAAtomicConnectorInstance> connectors, EMAMBluePrintCPP bluePrint, Method method) {
        for (EMAAtomicConnectorInstance connector : connectors) {
            if (connector.isConstant()) {
                if (connector.getSourcePort().isConstant()) {
                    EMAPortInstanceSymbol constPort = connector.getSourcePort();
                    Variable v1 = new Variable();
                    v1.setName(constPort.getConstantValue().get().getValueAsString());
                    String targetName = NameHelper.calculatePartialName(connector.getTargetPort(),
                            currentComponentSymbol);
                    Variable v2 = PortConverter.getVariableForPortSymbol(connector, targetName, bluePrint);


                    Instruction instruction = new ConnectInstructionCPP(v2, v1);
                    method.addInstruction(instruction);
                } else if (connector.getTargetPort().isConstant()) {
                    EMAPortInstanceSymbol constPort = connector.getTargetPort();
                    Variable v2 = new Variable();
                    v2.setName(constPort.getConstantValue().get().getValueAsString());
                    String sourceName = NameHelper.calculatePartialName(connector.getSourcePort(),
                            currentComponentSymbol);
                    Variable v1 = PortConverter.getVariableForPortSymbol(connector, sourceName, bluePrint);


                    Instruction instruction = new ConnectInstructionCPP(v2, v1);
                    method.addInstruction(instruction);
                } else {
                    Log.error("0xWRONGCONNECTOR the connector is constant but target nor source are constant");
                }
            }
        }
    }

    private static List<Instruction> createTargetConnectors(EMAMBluePrintCPP bluePrint,
                                                            EMAPortInstanceSymbol source) {
        List<Instruction> newInstructions = new LinkedList<>();
        for (EMAPortInstanceSymbol target : ConnectionHelper.targetsOf(source)) {
            if (target.equals(source)) continue;
            String sourceName = NameHelper.calculatePartialName(source,
                    currentComponentSymbol);
            String targetName = NameHelper.calculatePartialName(target,
                    currentComponentSymbol);
            Variable v1 = PortConverter.convertPortSymbolToVariable(source, sourceName, bluePrint);
            Variable v2 = PortConverter.convertPortSymbolToVariable(target, targetName, bluePrint);
            if (!currentComponentSymbol.getIncomingPortInstances().contains(source))
                v1.addAdditionalInformation(Variable.CROSSCOMPONENT);
            if (!currentComponentSymbol.getOutgoingPortInstances().contains(target))
                v2.addAdditionalInformation(Variable.CROSSCOMPONENT);

            Log.info("v1: " + v1.getName() + " v2: " + v2.getName(), "Variable Info:");
            Log.info("v1: " + v1.getNameTargetLanguageFormat() + " v2: " + v2.getNameTargetLanguageFormat(), "Variable Info:");

            Instruction instruction = new ConnectInstructionCPP(v2, v1);
            newInstructions.add(instruction);
        }
        return newInstructions;
    }

    private static List<MathExpressionSymbol> visitedMathExpressionSymbols = new ArrayList<>();
    private static boolean swapNextInstructions = false;

    private static void handleMathStatementGeneration(Method method, EMAMBluePrintCPP bluePrint,
                                                      MathStatementsSymbol mathStatementsSymbol,
                                                      GeneratorCPP generatorCPP, List<String> includeStrings) {
        // add math implementation instructions to method
        List<MathExpressionSymbol> newMathExpressionSymbols = new ArrayList<>();
        MathOptimizer.currentBluePrint = bluePrint;
        int counter = 0;
        visitedMathExpressionSymbols.clear();
        //int lastIndex = 0;
        for (currentGenerationIndex = 0; currentGenerationIndex < mathStatementsSymbol.getMathExpressionSymbols().size(); ++currentGenerationIndex) {
            int beginIndex = currentGenerationIndex;
            MathExpressionSymbol mathExpressionSymbol = mathStatementsSymbol.getMathExpressionSymbols().get(currentGenerationIndex);
            if (!containsExactObject(visitedMathExpressionSymbols, mathExpressionSymbol)) {
                if (generatorCPP.useAlgebraicOptimizations()) {
                    List<MathExpressionSymbol> precedingExpressions = new ArrayList<>();
                    for (int i = 0; i < counter; ++i)
                        precedingExpressions.add(mathStatementsSymbol.getMathExpressionSymbols().get(i));
                    if (mathExpressionSymbol != visitedMathExpressionSymbols)
                        newMathExpressionSymbols.add(MathOptimizer.applyOptimizations(mathExpressionSymbol, precedingExpressions, mathStatementsSymbol));
                    ++counter;
                }
                generateInstruction(method, mathExpressionSymbol, bluePrint, includeStrings);
                //lastIndex = currentGenerationIndex;
            }
            handleInstructionReOrdering(method, beginIndex);
        }
        if (generatorCPP.useAlgebraicOptimizations())
            removeUselessVariableDefinitions(method);
    }

    private static boolean containsExactObject(Collection collection, Object obj) {
        for (Object other : collection) {
            //equals is not used on purpose!
            //the jvm object id must be the same!
            if (other == obj) return true;
        }

        return false;
    }

    private static void removeUselessVariableDefinitions(Method method) {
        List<Instruction> instructionsToRemove = new ArrayList<>();
        for (Instruction instruction1 : method.getInstructions()) {
            if (instruction1 instanceof TargetCodeMathInstruction) {
                TargetCodeMathInstruction targetIns = (TargetCodeMathInstruction) instruction1;
                if (targetIns.getAddedVariable().isPresent()) {
                    boolean usesVariable = false;
                    for (Instruction instruction2 : method.getInstructions()) {
                        if (instruction2 instanceof TargetCodeMathInstruction) {
                            //Log.info("test", "removeUselessVariable:");
                            if (((TargetCodeMathInstruction) instruction2).getUsedVariables().contains(
                                    targetIns.getAddedVariable().get())) {
                                usesVariable = true;
                                break;
                            }
                        }
                    }
                    if (!usesVariable) {
                        instructionsToRemove.add(instruction1);
                    }
                }
            }
        }

        for (Instruction instructionToRemove : instructionsToRemove) {
            method.getInstructions().remove(instructionToRemove);
            Log.info(instructionToRemove.getTargetLanguageInstruction(), "Removed Instruction:");
        }
        if (instructionsToRemove.size() > 0) {
            removeUselessVariableDefinitions(method);
        }
    }

    private static void generateInstruction(Method method, MathExpressionSymbol
            mathExpressionSymbol, EMAMBluePrintCPP bluePrint, List<String> includeStrings/*, int lastIndex*/) {
//        MathFunctionFixer.fixMathFunctions(mathExpressionSymbol, bluePrint);
        String result = ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, includeStrings);
        String outputName = "";
        for (MathCommand mathCommand : ComponentConverter.usedMathCommand) {
            if (mathCommand != null) {
                String argumentNoReturnFunctionName = mathCommand.getMathCommandName();
                if (mathCommand.isArgumentNoReturnMathCommand() && result.contains(argumentNoReturnFunctionName)) {
                    outputName = getNameOfOutput(mathExpressionSymbol);
                    result = fixArgumentNoReturnInstruction(result, outputName);
                    //TODO Add fixType Here ---> think about put the function here or down
                }
            }
            //fixVariableTypes(mathCommand, result,mathExpressionSymbol, bluePrint);
        }
        TargetCodeMathInstruction instruction = new TargetCodeMathInstruction(result, mathExpressionSymbol);
        Log.info(mathExpressionSymbol.getClass().getName() + " " + mathExpressionSymbol.getTextualRepresentation(), "GenerateSymbol:");
        if (mathExpressionSymbol instanceof MathValueSymbol) {
            instruction.setAddedVariable(mathExpressionSymbol.getName());
        }
        calculateUsedVariables(mathExpressionSymbol, instruction);
        Log.info(instruction.getTargetCode(), "Instruction:");
        for (String name : instruction.getUsedVariables()) {
            Log.info(name, "Used Variables:");
        }
        method.addInstruction(instruction);
        visitedMathExpressionSymbols.add(mathExpressionSymbol);
        //Log.debug("lastIndex: " + lastIndex + " current: " + currentGenerationIndex, "ComponentConverterMethodGeneration");
    }

    private static void calculateUsedVariables(MathExpressionSymbol
                                                       mathExpressionSymbol, TargetCodeMathInstruction instruction) {
        if (mathExpressionSymbol != null) {
            Log.info(mathExpressionSymbol.getTextualRepresentation(), "EXP:");
        }
        if (mathExpressionSymbol == null) {
        } else if (mathExpressionSymbol instanceof MathValueSymbol) {
            calculateUsedVariables((MathValueSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isAssignmentExpression()) {
            calculateUsedVariables((MathAssignmentExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isArithmeticExpression()) {
            calculateUsedVariables((MathArithmeticExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isMatrixExpression()) {
            calculateUsedVariables((MathMatrixExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isValueExpression()) {
            if (((MathValueExpressionSymbol) mathExpressionSymbol).isNameExpression()) {
                calculateUsedVariables((MathNameExpressionSymbol) mathExpressionSymbol, instruction);
            }
        } else if (mathExpressionSymbol.isParenthesisExpression()) {
            calculateUsedVariables(mathExpressionSymbol.getRealMathExpressionSymbol(), instruction);
        } else if (mathExpressionSymbol.isForLoopExpression()) {
            calculateUsedVariables((MathForLoopExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol instanceof MathStringExpression) {
            for (MathExpressionSymbol expressionSymbol : ((MathStringExpression) mathExpressionSymbol).getPreviousExpressionSymbols())
                calculateUsedVariables(expressionSymbol, instruction);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName() + " " +
                    mathExpressionSymbol.getTextualRepresentation(), "Not handled calculateUsedVariables1:");
        }
    }

    private static void calculateUsedVariables(MathForLoopExpressionSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        calculateUsedVariables(mathExpressionSymbol.getForLoopHead().getMathExpression(), instruction);
        for (MathExpressionSymbol mathExpressionSymbol1 : mathExpressionSymbol.getForLoopBody()) {
            calculateUsedVariables(mathExpressionSymbol1, instruction);
        }
    }

    private static void calculateUsedVariables(MathMatrixExpressionSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        if (mathExpressionSymbol.isMatrixNameExpression()) {
            calculateUsedVariables((MathMatrixNameExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isMatrixArithmeticExpression()) {
            calculateUsedVariables((MathMatrixArithmeticExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isMatrixAccessExpression()) {
            calculateUsedVariables((MathMatrixAccessSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isMatrixVectorExpression()) {
            calculateUsedVariables((MathMatrixVectorExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol instanceof MathMatrixAccessOperatorSymbol) {
            calculateUsedVariables((MathMatrixAccessOperatorSymbol) mathExpressionSymbol, instruction);
            //calculateUsedVariables((MathMatrixPreOperatorExpressionSymbol) mathExpressionSymbol, instruction);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName() + " " +
                    mathExpressionSymbol.getTextualRepresentation(), "Not handled calculateUsedVariables2:");
        }
    }

    private static void calculateUsedVariables(MathMatrixNameExpressionSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        instruction.addUsedVariable(mathExpressionSymbol.getNameToAccess());
        calculateUsedVariables(mathExpressionSymbol.getMathMatrixAccessOperatorSymbol(), instruction);
    }

    private static void calculateUsedVariables(MathMatrixAccessOperatorSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        //parent calculateUsedVariables(mathExpressionSymbol.getMathMatrixNameExpressionSymbol(), instruction);
        for (MathExpressionSymbol expSymbol : mathExpressionSymbol.getMathMatrixAccessSymbols())
            calculateUsedVariables(expSymbol, instruction);
    }


    private static void calculateUsedVariables(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        calculateUsedVariables(mathExpressionSymbol.getLeftExpression(), instruction);
        calculateUsedVariables(mathExpressionSymbol.getRightExpression(), instruction);
    }


    private static void calculateUsedVariables(MathMatrixAccessSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        if (mathExpressionSymbol.getMathExpressionSymbol().isPresent()) {
            calculateUsedVariables(mathExpressionSymbol.getMathExpressionSymbol().get(), instruction);
        }
    }


    private static void calculateUsedVariables(MathMatrixVectorExpressionSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        calculateUsedVariables(mathExpressionSymbol.getStart(), instruction);
        if (mathExpressionSymbol.getStep().isPresent())
            calculateUsedVariables(mathExpressionSymbol.getStep().get(), instruction);
        calculateUsedVariables(mathExpressionSymbol.getEnd(), instruction);
    }

    private static void calculateUsedVariables(MathValueSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        if (mathExpressionSymbol.getValue() != null)
            calculateUsedVariables(mathExpressionSymbol.getValue().getRealMathExpressionSymbol(), instruction);
    }

    private static void calculateUsedVariables(MathArithmeticExpressionSymbol
                                                       mathExpressionSymbol, TargetCodeMathInstruction instruction) {
        calculateUsedVariables(mathExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), instruction);
        calculateUsedVariables(mathExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), instruction);
    }

    private static void calculateUsedVariables(MathAssignmentExpressionSymbol
                                                       mathExpressionSymbol, TargetCodeMathInstruction instruction) {
        calculateUsedVariables(mathExpressionSymbol.getExpressionSymbol(), instruction);
    }

    private static void calculateUsedVariables(MathNameExpressionSymbol
                                                       mathExpressionSymbol, TargetCodeMathInstruction instruction) {
        instruction.addUsedVariable(mathExpressionSymbol.getNameToResolveValue());
    }

    private static void handleInstructionReOrdering(Method method, int beginIndex) {
        if (swapNextInstructions) {
            swapNextInstructions = false;
            //Log.error("ad");
            Instruction lastInstruction = method.getInstructions().get(currentGenerationIndex);
            method.getInstructions().remove(currentGenerationIndex);
            method.addInstruction(lastInstruction);
        }
        if (beginIndex != currentGenerationIndex) swapNextInstructions = true;
    }

    private static String fixArgumentNoReturnInstruction(String instruction, String outputName) {
        String newInstruction = "";
        if (instruction.contains("=")) {
            int indexOfEqualOperator = instruction.indexOf("=");
            String afterEqualOperatorSubString = instruction.substring(indexOfEqualOperator + 2);

            if (afterEqualOperatorSubString.contains(",")) {
                int indexOfCommaOperator = afterEqualOperatorSubString.indexOf(",");
                newInstruction = afterEqualOperatorSubString.substring(0, indexOfCommaOperator) + ", " + outputName +
                        afterEqualOperatorSubString.substring(indexOfCommaOperator);
            } else {
                int indexOfBracket = afterEqualOperatorSubString.indexOf(")");
                newInstruction = afterEqualOperatorSubString.substring(0, indexOfBracket) + ", " + outputName +
                        afterEqualOperatorSubString.substring(indexOfBracket);
            }
            newInstruction = removeBracket(newInstruction);
            return newInstruction;
        }
        return instruction;
    }

    private static String removeBracket(String instruction) {
        String newInstruction = "";
        if (instruction.indexOf("(") == 0) {
            int indexOfLastBracket = instruction.lastIndexOf(")");
            newInstruction = instruction.substring(1, indexOfLastBracket) + instruction.substring(indexOfLastBracket + 1);
            return newInstruction;
        }
        return instruction;
    }


    public static Method generateOutputMethod(EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint,
                                              MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP,
                                              List<String> includeStrings) {
        Optional<Method> execute = bluePrint.getMethod(EXECUTE_METHOD_NAME);
        if (!execute.isPresent())
            return null;

        Method output = new Method(OUTPUT_METHOD_NAME, "void");
        output.setPublic(true);
        for (Variable parameter : execute.get().getParameters())
            output.addParameter(parameter);

        List<Instruction> newInstructions = new ArrayList<>();
        if (mathStatementsSymbol != null) {
            Map<String, String> resets = new HashMap<>();

            // copy static variables to temp variables
            for (Instruction instruction : execute.get().getInstructions()) {
                if (instruction instanceof TargetCodeMathInstruction) {
                    MathExpressionSymbol expressionSymbol = ((TargetCodeMathInstruction) instruction).getMathExpressionSymbol();
                    Optional<MathValueSymbol> copy = GenerateComponentFunction.createCopy(expressionSymbol, resets);
                    if (copy.isPresent())
                        newInstructions.add(new TargetCodeMathInstruction(copy.get().getTextualRepresentation() + "\n", copy.get()));
                }
                newInstructions.add(instruction);
            }
            // reset static variables
            for (Map.Entry<String, String> reset : resets.entrySet()) {
                MathAssignmentExpressionSymbol resetAssignment =
                        GenerateComponentFunction.createReset(reset.getValue(), reset.getKey());
                newInstructions.add(new TargetCodeMathInstruction(resetAssignment.getTextualRepresentation() + ";\n", resetAssignment));
            }
        } else {
            // do not update
            for (Instruction instruction : execute.get().getInstructions()) {
                if (instruction instanceof UpdateInstruction) continue;
                else if (instruction instanceof ExecuteInstruction && !(instruction instanceof OutputInstruction))
                    newInstructions.add(new OutputInstruction((ExecuteInstruction) instruction));
                else
                    newInstructions.add(instruction);
            }
        }

        output.setInstructions(newInstructions);
        return output;
    }

    public static Method generateUpdateMethod(EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint,
                                              MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP,
                                              List<String> includeStrings) {
        Optional<Method> execute = bluePrint.getMethod(EXECUTE_METHOD_NAME);
        if (!execute.isPresent())
            return null;

        Method update = new Method(UPDATE_METHOD_NAME, "void");
        update.setPublic(true);
        for (Variable parameter : execute.get().getParameters())
            update.addParameter(parameter);

        if (mathStatementsSymbol != null)
            update.setInstructions(execute.get().getInstructions());
        else {
            for (Instruction instruction : execute.get().getInstructions()) {
                if (instruction instanceof ExecuteInstruction && !(instruction instanceof OutputInstruction))
                    update.addInstruction(new UpdateInstruction((ExecuteInstruction) instruction));
            }
        }

        return update;
    }
}
