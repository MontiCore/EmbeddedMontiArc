/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.ConnectInstruction;
import de.monticore.lang.monticar.generator.ExecuteInstruction;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.instruction.ExecuteDynamicConnects;
import de.monticore.lang.monticar.generator.order.ImplementExecutionOrder;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.util.*;

/**
 */
public class ExecutionOrderFixer {
    public static void fixExecutionOrder(TaggingResolver taggingResolver, EMAMBluePrintCPP bluePrintCPP, GeneratorCPP generatorCPP) {

        Method method = bluePrintCPP.getMethod("execute").get();


        Map<String, List<Instruction>> map = new HashMap<>();

        List<EMAComponentInstanceSymbol> threadableSubComponents = bluePrintCPP.getOriginalSymbol().getIndependentSubComponents();
        List<Instruction> otherInstructions = computeOtherInstructions(map, method);
        List<EMAComponentInstanceSymbol> exOrder = ImplementExecutionOrder.exOrder(taggingResolver, bluePrintCPP.getOriginalSymbol());
        List<Instruction> newList = getExecutionOrderInstructionsList(exOrder, map, bluePrintCPP, threadableSubComponents);
        fixSlistExecutionOrder(bluePrintCPP.getOriginalSymbol(), newList, bluePrintCPP, threadableSubComponents, generatorCPP);
        List<TargetCodeInstruction> joinInstructions = new ArrayList<>();

        //if (generatorCPP.useThreadingOptimizations())
        for (Instruction instruction : newList) {
            if (instruction.isExecuteInstruction()) {
                ExecuteInstruction executeInstruction = (ExecuteInstruction) instruction;
                if (executeInstruction.canBeThreaded()) {
                    joinInstructions.add(new TargetCodeInstruction(executeInstruction.getThreadName() + ".join();\n"));
                }
            }
        }

        newList.addAll(joinInstructions);
        newList.addAll(otherInstructions);

        fixDynamicInstruction(newList, bluePrintCPP);
        fixNextInstruction(newList, bluePrintCPP);

        fixExecuteDynamicConnects(newList);

        method.setInstructions(newList);


    }

    public static void fixSlistExecutionOrder(EMAComponentInstanceSymbol instanceSymbol, List<Instruction> newList, EMAMBluePrintCPP bluePrintCPP, List<EMAComponentInstanceSymbol> threadableComponents, GeneratorCPP generatorCPP) {
        for (EMAComponentInstanceSymbol subComponent : instanceSymbol.getSubComponents()) {
            if (!listContainsExecuteInstruction(newList, subComponent.getName())) {
                ExecuteInstruction executeInstruction = (ExecuteInstruction) getExecuteInstruction(subComponent.getName(), bluePrintCPP, threadableComponents, generatorCPP);

                if(subComponent instanceof EMADynamicComponentInstanceSymbol) {
                    executeInstruction.setDynamic(((EMADynamicComponentInstanceSymbol) subComponent).isDynamicInstance());
                }
                int insertionIndex = getIndexOfLastConnectInstruction(newList, subComponent.getName());
                newList.add(insertionIndex, executeInstruction);
            }
        }
    }

    public static void fixNextInstruction(List<Instruction> newList, EMAMBluePrintCPP bluePrintCPP){
        if(bluePrintCPP.getMethod("next").isPresent()){
            newList.add(0, new TargetCodeInstruction("next();\n"));
        }
    }

    public static void fixDynamicInstruction(List<Instruction> newList, EMAMBluePrintCPP bluePrintCPP){
        if(bluePrintCPP.getMethod("dynamic").isPresent()){
            newList.add(0, new TargetCodeInstruction("dynamic();\n"));
        }
    }

    public static boolean listContainsExecuteInstruction(List<Instruction> list, String name) {
        for (Instruction instruction : list) {
            if (instruction.isExecuteInstruction()) {
                ExecuteInstruction executeInstruction = (ExecuteInstruction) instruction;
                if (executeInstruction.getComponentName().equals(GeneralHelperMethods.getTargetLanguageVariableInstanceName(name)))
                    return true;
            }
        }
        return false;
    }

    public static Instruction getExecuteInstruction(String nameToAdd, EMAMBluePrintCPP bluePrintCPP, List<EMAComponentInstanceSymbol> threadableComponents, GeneratorCPP generatorCPP) {
        boolean canBeThreaded = false;
        if (generatorCPP.useThreadingOptimizations())
            for (EMAComponentInstanceSymbol instanceSymbol : threadableComponents) {
                if (nameToAdd.equals(instanceSymbol.getName()))
                    canBeThreaded = true;
            }
        String name = GeneralHelperMethods.getTargetLanguageComponentName(nameToAdd);
        Log.info(name, "Adding ExecuteInstruction:");
        ExecuteInstruction exI = new ExecuteInstruction(name, bluePrintCPP, canBeThreaded);
        return exI;
    }

    public static Instruction getExecuteInstruction(EMAComponentInstanceSymbol componentInstanceSymbol, EMAMBluePrintCPP bluePrintCPP, List<EMAComponentInstanceSymbol> threadableComponents) {
        return getExecuteInstruction(componentInstanceSymbol.getName(), bluePrintCPP, threadableComponents, (GeneratorCPP) bluePrintCPP.getGenerator());
    }

    public static List<Instruction> computeOtherInstructions(Map<String, List<Instruction>> map, Method method) {
        List<Instruction> otherInstructions = new ArrayList<>();
        for (Instruction instruction : method.getInstructions()) {
            if (instruction.isConnectInstruction()) {
                ConnectInstruction connectInstruction = (ConnectInstruction) instruction;
                if (connectInstruction.getVariable1().isCrossComponentVariable()) {
                    String name = connectInstruction.getVariable1().getComponentName();
                    Log.info(name, "ComponentName:");
                    if (map.containsKey(name)) {
                        map.get(name).add(instruction);
                    } else {
                        List<Instruction> list = new ArrayList<Instruction>();
                        list.add(instruction);
                        map.put(name, list);
                    }
                } else {
                    otherInstructions.add(instruction);
                }
            } else {
                if(instruction.isTargetCodeInstruction() && instruction.getTargetLanguageInstruction().equals("next();\n")){
                    continue;
                }

                if(instruction.isTargetCodeInstruction() && (instruction instanceof ExecuteDynamicConnects)){
                    ExecuteDynamicConnects edc = (ExecuteDynamicConnects)instruction;
                    if(edc.getBeforeComponent().isPresent()){
                        if(!map.containsKey(edc.getBeforeComponentName())){
                            List<Instruction> l = new ArrayList<>();
                            map.put(edc.getBeforeComponentName(),l);
                        }
                        map.get(edc.getBeforeComponentName()).add(instruction);
                        continue;
                    }

                }

                otherInstructions.add(instruction);
            }
        }
        return otherInstructions;
    }

    public static List<Instruction> getExecutionOrderInstructionsList(List<EMAComponentInstanceSymbol> exOrder, Map<String,
            List<Instruction>> map, EMAMBluePrintCPP bluePrintCPP, List<EMAComponentInstanceSymbol> threadableSubComponents) {
        List<Instruction> newList = new ArrayList<>();
        for (EMAComponentInstanceSymbol instanceSymbol : exOrder) {
            String namey = instanceSymbol.getName();
            Log.info(namey, "Trying to add:");
            if (map.containsKey(namey)) {
                for (Instruction i : map.get(namey))
                    if (!newList.contains(i))
                        newList.add(i);
                Log.info(namey, "Added Namey:");
            }
            boolean add = bluePrintCPP.getOriginalSymbol().isSubComponent(instanceSymbol.getFullName());
            if (add) {
                ExecuteInstruction executeInstruction = (ExecuteInstruction) getExecuteInstruction(instanceSymbol, bluePrintCPP, threadableSubComponents);
                if (!newList.contains(executeInstruction)) {
                    if(instanceSymbol instanceof EMADynamicComponentInstanceSymbol){
                        executeInstruction.setDynamic(((EMADynamicComponentInstanceSymbol) instanceSymbol).isDynamicInstance());
                    }
                    newList.add(executeInstruction);
                }
            }
        }
        for (EMAComponentInstanceSymbol subComponent : bluePrintCPP.getOriginalSymbol().getSubComponents()) {
            String namey = subComponent.getName();
            if (map.containsKey(namey)) {
                List<Instruction> instructionsToAdd = map.get(namey);
                for (Instruction i : instructionsToAdd) {
                    if (!newList.contains(i))
                        newList.add(i);
                }
                Log.info(namey, "Added Namey:");
            }
        }
        return newList;
    }

    public static void fixExecuteDynamicConnects(List<Instruction> newList){

        List<Integer> idx = new ArrayList<>();

        for(int i = 1; i < newList.size(); ++i){
            if((newList.get(i-1) instanceof ExecuteDynamicConnects) && (newList.get(i) instanceof ExecuteDynamicConnects)){
                ExecuteDynamicConnects a = (ExecuteDynamicConnects) newList.get(i-1);
                ExecuteDynamicConnects b = (ExecuteDynamicConnects) newList.get(i);
                if(a.getBeforeComponentName().equals(b.getBeforeComponentName())){
                    idx.add(i);
                }
            }
        }

        Collections.sort(idx, Collections.reverseOrder());
        for(int i : idx){
            newList.remove(i);
        }

    }

    private static int getIndexOfLastConnectInstruction(List<Instruction> instructions, String componentInstanceName) {
        int result = -1;
        for (int i = 0, len = instructions.size(); i < len; i++) {
            Instruction instr = instructions.get(i);
            if (instr.isConnectInstruction()) {
                Variable lhs = ((ConnectInstruction) instr).getVariable1();
                if (lhs.getName().startsWith(componentInstanceName) && i > result) {
                    result = i;
                }
            }
        }
        return result + 1;
    }
}
