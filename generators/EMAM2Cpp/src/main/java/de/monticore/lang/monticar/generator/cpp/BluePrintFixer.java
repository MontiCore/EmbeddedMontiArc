/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.EMAMBluePrint;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

/**
 */
public class BluePrintFixer {
    public static void fixBluePrintVariableArrays(EMAMBluePrint bluePrint) {
        List<Variable> newVars = new ArrayList<>();

        //Group variables of the same array
        //Ports that are not part of an array are handled as array with size 1
        Map<String, List<Variable>> nameToVariable = bluePrint.getVariables().stream()
                .collect(Collectors.groupingBy(Variable::getNameWithoutArrayNamePart));

        //Used to keep the original order
        List<String> orderedUniqueNames = bluePrint.getVariables().stream()
                .map(Variable::getNameWithoutArrayNamePart)
                .distinct()
                .collect(Collectors.toList());

        //Only keep one and set the right array size
        orderedUniqueNames.forEach((nameWithoutArray) -> {
            List<Variable> varList = nameToVariable.get(nameWithoutArray);
            Variable firstVar = varList.get(0);
            firstVar.setName(nameWithoutArray);
            firstVar.setArraySize(varList.size());
            newVars.add(firstVar);

            boolean isDynamic = false;

            for(Variable v : varList){
                isDynamic = isDynamic | v.isDynamic();
            }
            if(isDynamic){
                newVars.add(addConnectedVariableForVariable(varList, nameWithoutArray, bluePrint));
                //newVars.add(addConnectedRequestQueueForVariable(nameWithoutArray, bluePrint));
            }
        });


        bluePrint.setVariables(newVars);
    }


    protected static Variable addConnectedVariableForVariable(List<Variable> varList, String nameWithoutArray, EMAMBluePrint bluePrint){
        Log.info("Adding __connected variable for "+nameWithoutArray, "Dynamic Connected Variable");

        VariableConstantArray variable = new VariableConstantArray("__"+nameWithoutArray+"_connected");

        variable.addAdditionalInformation(Variable.ORIGINPORT);
        variable.setArraySize(varList.size());
        variable.setTypeNameTargetLanguage("bool");
//        variable.setIsConstantVariable(true);
//        variable.setConstantValue(s);

        variable.setPublic(false);
        for(int i = 0; i < varList.size(); ++i){
            variable.addConstantInitValue(varList.get(i).isDynamic() ? "false" : "true");
        }

        bluePrint.getMathInformationRegister().addVariable(variable);

        return variable;
    }


    public static void fixBluePrintDynamicVariableConnectRequestQueues(EMAMBluePrint bluePrint){
        int s = bluePrint.getVariables().size();
        for(int i = 0; i < s; ++i){
            Variable v = bluePrint.getVariables().get(i);
            if(v.isDynamic()){

                if(!bluePrint.getVariable(String.format("__%s_connect_request", v.getNameWithoutArrayNamePart())).isPresent()){
                    bluePrint.addVariable(addConnectedRequestQueueForVariable(v.getNameWithoutArrayNamePart(), bluePrint));
                }
                if(!bluePrint.getVariable(String.format("__%s_free_request", v.getNameWithoutArrayNamePart())).isPresent()){
                    bluePrint.addVariable(addFreeRequestQueueForVariable(v.getNameWithoutArrayNamePart(), bluePrint));
                }


                if(!bluePrint.getMethod(v.getNameWithoutArrayNamePart()+"_has_connect_request").isPresent()){
                    Method m = new Method();
                    m.setName(v.getNameWithoutArrayNamePart()+"_has_connect_request");
                    m.setReturnTypeName("bool");
                    m.addInstruction(new TargetCodeInstruction(String.format("return !__%s_connect_request.empty();\n", v.getNameWithoutArrayNamePart())));
                    bluePrint.addMethod(m);
                }

                if(!bluePrint.getMethod(v.getNameWithoutArrayNamePart()+"_connect_request_front").isPresent()) {
                    Method m = new Method();
                    m.setName(v.getNameWithoutArrayNamePart()+"_connect_request_front");
                    m.setReturnTypeName("int");

                    m.addInstruction(new TargetCodeInstruction(String.format("int r = __%s_connect_request.front();\n", v.getNameWithoutArrayNamePart())));
                    m.addInstruction(new TargetCodeInstruction(String.format("__%s_connect_request.pop();\n", v.getNameWithoutArrayNamePart())));
                    m.addInstruction(new TargetCodeInstruction("return r;\n"));
                    bluePrint.addMethod(m);
                }

                // free part

                if(!bluePrint.getMethod(v.getNameWithoutArrayNamePart()+"_has_free_request").isPresent()){
                    Method m = new Method();
                    m.setName(v.getNameWithoutArrayNamePart()+"_has_free_request");
                    m.setReturnTypeName("bool");
                    m.addInstruction(new TargetCodeInstruction(String.format("return !__%s_free_request.empty();\n", v.getNameWithoutArrayNamePart())));
                    bluePrint.addMethod(m);
                }

                if(!bluePrint.getMethod(v.getNameWithoutArrayNamePart()+"_free_request_front").isPresent()) {
                    Method m = new Method();
                    m.setName(v.getNameWithoutArrayNamePart()+"_free_request_front");
                    m.setReturnTypeName("int");

                    m.addInstruction(new TargetCodeInstruction(String.format("int r = __%s_free_request.front();\n", v.getNameWithoutArrayNamePart())));
                    m.addInstruction(new TargetCodeInstruction(String.format("__%s_free_request.pop();\n", v.getNameWithoutArrayNamePart())));
                    m.addInstruction(new TargetCodeInstruction(String.format("__%s_connected[r] = false;\n", v.getNameWithoutArrayNamePart())));
                    m.addInstruction(new TargetCodeInstruction("return r;\n"));
                    bluePrint.addMethod(m);
                }

                if(!bluePrint.getMethod(v.getNameWithoutArrayNamePart()+"_free_request_front_peak").isPresent()) {
                    Method m = new Method();
                    m.setName(v.getNameWithoutArrayNamePart()+"_free_request_front_peak");
                    m.setReturnTypeName("int");

                    m.addInstruction(new TargetCodeInstruction(String.format("return __%s_free_request.front();\n", v.getNameWithoutArrayNamePart())));
                    bluePrint.addMethod(m);
                }
            }
        }
    }

    protected static Variable addConnectedRequestQueueForVariable(String nameWithoutArray, EMAMBluePrint bluePrint){
        Log.info("Adding __connect_request variable for "+nameWithoutArray, "Dynamic Request Connect Queue for Variable");
        Variable variable = new Variable();
        variable.setName("__"+nameWithoutArray+"_connect_request");
        variable.setTypeNameTargetLanguage("std::queue<int>");

        variable.setPublic(false);

        bluePrint.getMathInformationRegister().addVariable(variable);

        return variable;
    }

    protected static Variable addFreeRequestQueueForVariable(String nameWithoutArray, EMAMBluePrint bluePrint){
        Log.info("Adding __free_request variable for "+nameWithoutArray, "Dynamic Request Free Queue for Variable");
        Variable variable = new Variable();
        variable.setName("__"+nameWithoutArray+"_free_request");
        variable.setTypeNameTargetLanguage("std::queue<int>");

        variable.setPublic(false);

        bluePrint.getMathInformationRegister().addVariable(variable);

        return variable;
    }

}
