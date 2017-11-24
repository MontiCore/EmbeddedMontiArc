package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.monticore.lang.monticar.generator.order.ImplementExecutionOrder;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * @author Sascha Schneiders
 */
public class ExecutionOrderFixer {
    public static void fixExecutionOrder(TaggingResolver taggingResolver, BluePrintCPP bluePrintCPP, GeneratorCPP generatorCPP) {
        Method method = bluePrintCPP.getMethod("execute").get();
        Map<String, List<Instruction>> map = new HashMap<>();

        List<ExpandedComponentInstanceSymbol> threadableSubComponents = bluePrintCPP.getOriginalSymbol().getIndependentSubComponents();
        List<Instruction> otherInstructions = computeOtherInstructions(map, method);
        List<ExpandedComponentInstanceSymbol> exOrder = ImplementExecutionOrder.exOrder(taggingResolver, bluePrintCPP.getOriginalSymbol());
        List<Instruction> newList = getExecutionOrderInstructionsList(exOrder, map, bluePrintCPP, threadableSubComponents);
        fixSlistExecutionOrder(bluePrintCPP.getOriginalSymbol(), newList, bluePrintCPP, threadableSubComponents, generatorCPP);
        List<TargetCodeInstruction> joinInstructions = new ArrayList<>();

        //if (generatorCPP.useThreadingOptimizations())
        for (Instruction instruction : newList) {
            if (instruction.isExecuteInstruction()) {
                ExecuteInstruction executeInstruction = (ExecuteInstruction) instruction;
                if (executeInstruction.isCanBeThreaded()) {
                    joinInstructions.add(new TargetCodeInstruction(executeInstruction.getThreadName() + ".join();\n"));
                }
            }
        }

        newList.addAll(joinInstructions);
        newList.addAll(otherInstructions);
        method.setInstructions(newList);
    }

    public static void fixSlistExecutionOrder(ExpandedComponentInstanceSymbol instanceSymbol, List<Instruction> newList, BluePrintCPP bluePrintCPP, List<ExpandedComponentInstanceSymbol> threadableComponents, GeneratorCPP generatorCPP) {
        for (ExpandedComponentInstanceSymbol subComponent : instanceSymbol.getSubComponents()) {
            if (!listContainsExecuteInstruction(newList, subComponent.getName())) {
                ExecuteInstruction executeInstruction = (ExecuteInstruction) getExecuteInstruction(subComponent.getName(), bluePrintCPP, threadableComponents, generatorCPP);

                newList.add(executeInstruction);
            }
        }

        /*int lastDotIndex = instanceSymbol.getFullName().lastIndexOf(".");
        if (lastDotIndex != -1) {
            String name = instanceSymbol.getFullName().substring(0, lastDotIndex);
            //Log.info(name,"Trying to fix name:");
            if (parentSymbol.isSubComponent(name)) {
                //Log.info(name,"IsSubcomponent ExecuteInstruction:");
                String nameToAdd;
                if (!listContainsExecuteInstruction(newList, name)) {
                    //Log.info(name,"Added additional ExecuteInstruction:");
                    newList.add(getExecuteInstruction(name));
                }
            }
        }*/
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


    public static Instruction getExecuteInstruction(String nameToAdd, BluePrintCPP bluePrintCPP, List<ExpandedComponentInstanceSymbol> threadableComponents, GeneratorCPP generatorCPP) {
        boolean canBeThreaded = false;
        if (generatorCPP.useThreadingOptimizations())
            for (ExpandedComponentInstanceSymbol instanceSymbol : threadableComponents) {
                if (nameToAdd.equals(instanceSymbol.getName()))
                    canBeThreaded = true;
            }
        String name = GeneralHelperMethods.getTargetLanguageComponentName(nameToAdd);
        Log.info(name, "Adding ExecuteInstruction:");
        return new ExecuteInstruction(name, bluePrintCPP, canBeThreaded);
    }

    public static Instruction getExecuteInstruction(ExpandedComponentInstanceSymbol componentInstanceSymbol, BluePrintCPP bluePrintCPP, List<ExpandedComponentInstanceSymbol> threadableComponents) {
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
                otherInstructions.add(instruction);
            }
        }
        return otherInstructions;
    }

    public static List<Instruction> getExecutionOrderInstructionsList(List<ExpandedComponentInstanceSymbol> exOrder, Map<String, List<Instruction>> map, BluePrintCPP bluePrintCPP, List<ExpandedComponentInstanceSymbol> threadableSubComponents) {
        List<Instruction> newList = new ArrayList<>();
        for (ExpandedComponentInstanceSymbol instanceSymbol : exOrder) {
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
                if (!newList.contains(executeInstruction))
                    newList.add(executeInstruction);
            }
        }
        for (ExpandedComponentInstanceSymbol subComponent : bluePrintCPP.getOriginalSymbol().getSubComponents()) {
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
}
