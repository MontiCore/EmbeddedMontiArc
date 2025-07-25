/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicEventHandlerInstanceSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.instruction.EventConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.instruction.ExecuteDynamicConnects;
import de.se_rwth.commons.logging.Log;


import java.util.*;
import java.util.stream.Collectors;

public class EventDynamicConnectConverter {

    protected static final String DYNPORTID = "_%s_dynPortID";
    protected static final String THISCOMPONENTPORTREQUEST = "int "+DYNPORTID+" = __%s_connect_request.front(); __%s_connect_request.pop(); _connected_idxs[%d] = "+DYNPORTID+";\n";

    protected static final String THISCOMPONENTPORTREQUEST_2 = "int "+DYNPORTID+" = dynamicconnect(%d, __%s_connected, &__%s_connect_request); _connected_idxs[%d] = "+DYNPORTID+";\n";
    protected static final String THISCOMPONENTPORTREQUEST_2_2 = "if(__%s_connect_request.front() == "+DYNPORTID+"){ __%s_connect_request.pop(); }\n";


    protected static final String DYNPORTIDININSTANCE = "_%s_%s_dynPortID";
    protected static final String DYNPORTIDININSTANCEINIT = "int "+DYNPORTIDININSTANCE+" = -1;\n";
    protected static final String DYNPORTIDININSTANCEGETTER = "int "+DYNPORTIDININSTANCE+" = %s.%s_connect_request_front(); _connected_idxs[%d] = "+DYNPORTIDININSTANCE+";\n";
    protected static final String DYNINSTANCEID = "_%s_dynINSTID";
    protected static final String DYNINSTANCECONNECT = "int "+DYNINSTANCEID+" = dynamicconnect(%d, __%s_connected); if( "+DYNINSTANCEID+" < 0 ){ free(_connected_idxs); return; }  _connected_idxs[%d] = "+DYNINSTANCEID+";\n";

//    protected static final String FREE_CONNECTIONTRACARRAY = "__event_connects_%s";
//    protected static final String FREE_THISCOMPONENTPORTREQUEST = "int "+DYNPORTID+" = _connected_idxs[%d];\n";
    protected static final String FREE_THISCOMPONENTPORTREQUEST_CONNECTED_ARRAY = "__%s_connected["+DYNPORTID+"] = false;\n";
//    protected static final String FREE_DYNINSTANCECONNECT = "int "+DYNINSTANCEID+" = _connected_idxs[%d];\n";
    protected static final String FREE_DYNINSTANCECONNECT_CONNECTED_ARRAY =  "__%s_connected["+DYNINSTANCEID+"] = false;\n";
//    protected static final String FREE_DYNPORTIDININSTANCE = "int "+DYNPORTIDININSTANCE+" = _connected_idxs[%d];\n";

    protected static final String MFREE_PORTID = "int "+DYNPORTID+" = %s_free_request_front();\n";
    protected static final String MFREE_PORTIDININSTANCE = "int "+DYNPORTIDININSTANCE+" = %s.%s_free_request_front();\n";
    protected static final String MFREE_PORTID_FROM_ARRAY = "int "+DYNPORTID+" = _connected_idxs[%d]; __%s_free_request.push("+DYNPORTID+");\n";
    protected static final String MFREE_PORTID_FROM_ARRAY_2 = "if(__%s_free_request.front() == "+DYNPORTID+"){ __%s_free_request.pop(); }\n";
    protected static final String MFREE_DYNINSTID = "int "+DYNINSTANCEID+" = _connected_idxs[%d];\n";
    protected static final String MFREE_PORTIDININSTANCE_2 = "int "+DYNPORTIDININSTANCE+" = _connected_idxs[%d];\n";

    protected static final String MFREE_PORTID_COND = "("+DYNPORTID+" == _connected_idxs[%d])";
    protected static final String MFREE_PORTIDININSTANCE_COND = "("+DYNPORTIDININSTANCE+" == _connected_idxs[%d])";


    protected static final String EVENT_FREE_PORTID = "int "+DYNPORTID+" = %s_free_request_front_peak();\n";
    protected static final String EVENT_FREE_PORTIDININSTANCE  = "int "+DYNPORTIDININSTANCE+" = %s.%s_free_request_front_peak();\n";


    protected static int free_method_index_counter = 0;
    protected static int freePositionInCodeCounter = 0;
    protected static List<String> resetConnectedArray = new ArrayList<>();

    public static boolean generateDynamicConnectEvent(EMADynamicEventHandlerInstanceSymbol event, EMAComponentInstanceSymbol componentSymbol, Method executeMethod, EMAMBluePrintCPP bluePrint ) {

        free_method_index_counter = 0;
        resetConnectedArray.clear();

        List<String> names= new ArrayList<>();
        event.getCondition().getConnectPortNames(names);
        java.util.Collections.sort(names);


        List<String> portNamesNotInEventCondition = getNamesOfPortsWhichAreNotInEventCondition(event, names);

        List<String> newInstances = getNewInstances(event);
        Map<String, Map<String, Optional<EMADynamicConnectorInstanceSymbol>>>  newPortsInstances = getNewPortsOfInstances(event, names);

        generateConnectMethod(names, componentSymbol, bluePrint);
//        Method free = generateFreeMethod(names, event, bluePrint);


        String bodyname = "__event_body_"+event.getName().replace("[", "_").replace("]", "_");
        String bodynameFree = "__event_body_free_"+event.getName().replace("[", "_").replace("]", "_");
        Method body = new Method(bodyname, "void");
        body.setPublic(false);
        Method free = new Method(bodynameFree, "void");
        free.setPublic(false);


        generateDynamicMethod(bluePrint, bodyname, bodynameFree);
        freeGenerateEventCondition(free, bluePrint, names, event);

        body.addInstruction(new TargetCodeInstruction("while("+EventConnectInstructionCPP.getEventNameCPP(event.getName())+"()){\n"));

        generateHandleConnectRequestsOfQueues(names, body, free, event);
        generateHandleConnectRequestsOfThisPorts(portNamesNotInEventCondition, body, free, componentSymbol);
        generateHandleConnectRequestsOfDynamicInstances(newInstances, body, componentSymbol, free);
        generateHandleConnectRequestInInstances(newPortsInstances, body, free);


        generateConnects(event, body, bluePrint, executeMethod, free);

//        generateDummyConnects(event, componentSymbol, executeMethod, bluePrint);


        //finish everything that needs to be set
//
        body.getInstructions().add(1, new TargetCodeInstruction("int* _connected_idxs = (int *)calloc("+free_method_index_counter+", sizeof(int));\n"));
//TODO        generateEndOfFreeMethod(event, free, componentSymbol);

        free.addInstruction(new TargetCodeInstruction(
                "__event_connects_"+ convertName(event.getName())+".erase(__event_connects_"+ convertName(event.getName())+".begin()+i);\n"
        ));
        for(String s : resetConnectedArray){
            free.addInstruction(new TargetCodeInstruction(s));
        }
        free.addInstruction(new TargetCodeInstruction("free(_connected_idxs);\n"));
        free.addInstruction(new TargetCodeInstruction("}\n"));
        free.addInstruction(new TargetCodeInstruction("}\n"));
        free.addInstruction(new TargetCodeInstruction("}\n"));

        String varName = "__event_connects_"+convertName(event.getName());
        if(!bluePrint.getVariable(varName).isPresent()){
            Variable v = new Variable();
            v.setName(varName);
            v.setTypeNameTargetLanguage("std::vector<int*>");
            v.setPublic(false);
            bluePrint.addVariable(v);
        }
        body.addInstruction(new TargetCodeInstruction(varName+".push_back(_connected_idxs);\n"));

        body.addInstruction(new TargetCodeInstruction("}\n"));
        bluePrint.addMethod(body);

        bluePrint.addMethod(free);
        return true;
    }



    protected static void generateConnectMethod(List<String> portConnectNames, EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint){

        List<String> names = new ArrayList<>();
        for(String pn : portConnectNames){
            if(!pn.contains(".")){
                names.add(pn);
            }
        }

        if(names.size() == 0){
            return;
        }

        String name = "connect_"+String.join("_", names);
        if(bluePrint.getMethod(name).isPresent()){
            return;
        }

        Log.info("Create connect method: "+name+"(...)", "EventConverter");

        Method method = new Method(name, "bool");
        Method free_method = new Method("free_"+String.join("_", names), "bool");
        TargetCodeInstruction freeCheck = new TargetCodeInstruction();
        free_method.addInstruction(freeCheck);

        List<String> checks = new ArrayList<>();
        List<Instruction> valueInits = new ArrayList<>();
        List<String> freeChecks = new ArrayList<>();
        for(String n : names ){
            Variable v = new Variable();
            v.setName(n+"_indexref");
            v.setTypeNameTargetLanguage("int*");
            method.addParameter(v);

            Optional<EMAPortInstanceSymbol> port = componentSymbol.getIncomingPortInstance(n+"[1]");
            if(port.isPresent()){
                Variable v_initValue = new Variable();
                v_initValue.setName(n+"_init");
                v_initValue.setVariableType(TypeConverter.getVariableTypeForMontiCarTypeName(port.get().getTypeReference().getName(), v_initValue, port.get()).get());
                method.addParameter(v_initValue);

                valueInits.add(new TargetCodeInstruction(String.format("%1$s[*%1$s_indexref] = %1$s_init;\n", n)));
            }

            long counter = componentSymbol.getPortInstanceList().stream().filter(p->p.getNameWithoutArrayBracketPart().equals(n)).count();

            String inst = String.format("*%s_indexref = dynamicconnect(%d, __%s_connected, &__%s_connect_request);\n", n, counter, n,n);

            method.addInstruction(
                    new TargetCodeInstruction(inst)
            );

            checks.add(String.format("(*%s_indexref < 0)", n));

            //free part
            v = new Variable();
            v.setName(n+"_indexref");
            v.setTypeNameTargetLanguage("int");
            free_method.addParameter(v);

            freeChecks.add("("+n+"_indexref < 0)");
            freeChecks.add("("+counter+" <= "+n+"_indexref)");
            freeChecks.add("(!__"+n+"_connected["+n+"_indexref])");
            free_method.addInstruction(new TargetCodeInstruction("__"+n+"_free_request.push("+n+"_indexref);\n"));

        }
        method.addInstruction(new TargetCodeInstruction(
                String.format("if(%s){return false;}\n", String.join(" || ", checks))
        ));

        //add inits to method
        valueInits.stream().forEach(vi -> method.addInstruction(vi));

        method.addInstruction(new TargetCodeInstruction("return true;\n"));
        bluePrint.addAdditionalUserIncludeStrings("DynamicHelper");
        bluePrint.addMethod(method);

        free_method.addInstruction(new TargetCodeInstruction("dynamicfree();\n"));
        free_method.addInstruction(new TargetCodeInstruction("return true;\n"));
        freeCheck.setInstruction("if( "+String.join(" || ", freeChecks)+" ){ return false; }\n");
        bluePrint.addMethod(free_method);
    }

    //TODO: Free Methods

//    protected static Method generateFreeMethod(List<String> portConnectNames, EMADynamicEventHandlerInstanceSymbol event, EMAMBluePrintCPP bluePrint){
//
//        List<String> names = new ArrayList<>();
//        for(String pn : portConnectNames){
//            if(!pn.contains(".")){
//                names.add(pn);
//            }
//        }
//
////        if(names.size() == 0){
////            return null;
////        }
//
//
//        String name = "free_"+String.join("_", names);
//        Method method = null;
//        if(bluePrint.getMethod(name).isPresent()){
//            Log.info("Extend free method: "+name+"(...)", "EventConverter");
//            method = bluePrint.getMethod(name).get();
//        }else{
//            Log.info("Create free method: "+name+"(...)", "EventConverter");
//            method = new Method(name, "void");
//            for(String n : names){
//                Variable v = new Variable();
//                v.setName(n+"_Idx");
//                v.setTypeNameTargetLanguage("int");
//                method.addParameter(v);
//            }
//        }
//
//        method.addInstruction(new TargetCodeInstruction("for(long i = __event_connects_"+convertName(event.getName())+".size()-1; i >= 0; --i){\n"));
//        method.addInstruction(new TargetCodeInstruction("int* _connected_idxs = __event_connects_"+convertName(event.getName())+".at(i);\n"));
//        method.addInstruction(new TargetCodeInstruction("if( "));
//
//        for(int i = 0; i < names.size(); ++i){
//            method.addInstruction(new TargetCodeInstruction(
//                    "(_connected_idxs["+i+"] == "+names.get(i)+"_Idx)"
//            ));
//            if(i < names.size()-1){
//                method.addInstruction(new TargetCodeInstruction(" && "));
//            }
//        }
//        method.addInstruction(new TargetCodeInstruction(" ){\n"));
//
//        bluePrint.addMethod(method);
//        return method;
//    }

//    protected static void generateEndOfFreeMethod(EMADynamicEventHandlerInstanceSymbol event, Method free, EMAComponentInstanceSymbol componentSymbol){
//
////        if(free == null){
////            return;
////        }
//
//        String name = String.format(FREE_CONNECTIONTRACARRAY, convertName(event.getName()));
//        free.addInstruction(new TargetCodeInstruction(
//                name+".erase("+name+".begin()+i);\n"
//        ));
//        for(String s : resetConnectedArray){
//            free.addInstruction(new TargetCodeInstruction(s));
//        }
//        free.addInstruction(new TargetCodeInstruction("free(_connected_idxs);\n"));
//
////        free.addInstruction(new TargetCodeInstruction("//----------------------------------------------------------------------------\n"));
//        List<String> names = new ArrayList<>();
//        event.getCondition().getConnectPortNames(names);
//        for(EMADynamicEventHandlerInstanceSymbol freeEvent : getFreeEvents(componentSymbol)){
//            List<String> freeNames = new ArrayList<>();
//            freeEvent.getCondition().getFreePortNames(freeNames);
//            freeNames.removeAll(names);
//
//            if(freeNames.isEmpty()){
//                // free event
//                free.addInstruction(new TargetCodeInstruction("if("+EventConnectInstructionCPP.getEventNameCPP(freeEvent.getName())+"){\n"));
//                generateConnectsForFreeEventBody(freeEvent, free);
//                free.addInstruction(new TargetCodeInstruction("}\n"));
//            }
//        }
////        free.addInstruction(new TargetCodeInstruction("//----------------------------------------------------------------------------\n"));
//
//        free.addInstruction(new TargetCodeInstruction("}\n}\n"));
//    }

    protected static void freeGenerateEventCondition(Method free, EMAMBluePrint bluePrint, List<String> names, EMADynamicEventHandlerInstanceSymbol event){

        String cond = "";
        for(int i = 0; i < names.size(); ++i){
            cond += names.get(i)+"_has_free_request()";
            if( i < names.size()-1){
                cond += " && ";
            }
        }
        free.addInstruction(new TargetCodeInstruction("while("+cond+"){\n"));
//        free.addInstruction(new TargetCodeInstruction("for(long i = __event_connects_"+convertName(event.getName())+".size()-1; i >= 0; --i){\n"));
//        free.addInstruction(new TargetCodeInstruction("int* _connected_idxs = __event_connects_"+convertName(event.getName())+".at(i);\n"));
//        free.addInstruction(new TargetCodeInstruction("if( "));
//
//        for(int i = 0; i < names.size(); ++i){
//            method.addInstruction(new TargetCodeInstruction(
//                    "(_connected_idxs["+i+"] == "+names.get(i)+"_Idx)"
//            ));
//            if(i < names.size()-1){
//                method.addInstruction(new TargetCodeInstruction(" && "));
//            }
//        }
//        method.addInstruction(new TargetCodeInstruction(" ){\n"));
//        bluePrint.getMethod("free").get().addInstruction(new TargetCodeInstruction(";\n}\n"));
    }

    protected static void generateDynamicMethod(EMAMBluePrintCPP bluePrint, String eventBodyName, String eventBodyNameFree){
        Optional<Method> dynamic = bluePrint.getMethod("dynamic");
        if(!dynamic.isPresent()){
            Method d = new Method("dynamic", "void");
            d.setPublic(false);
            bluePrint.addMethod(d);
            dynamic = Optional.of(d);

            d = new Method("dynamicWrapper", "static void");
            Variable v = new Variable();
            v.setName("pt2Object");
            v.setTypeNameTargetLanguage("void*");
            d.addParameter(v);
            v = new Variable();
            v.setName("dynamicFunc");
            v.setTypeNameTargetLanguage("bool");
            d.addParameter(v);
            v = new Variable();
            v.setName("freeFunc");
            v.setTypeNameTargetLanguage("bool");
            d.addParameter(v);

            d.setPublic(false);
            d.addInstruction(new TargetCodeInstruction("if(dynamicFunc){(("+bluePrint.getName()+"*)pt2Object)->dynamic();}\n"));
            d.addInstruction(new TargetCodeInstruction("if(freeFunc){(("+bluePrint.getName()+"*)pt2Object)->dynamicfree();}\n"));
            bluePrint.addMethod(d);


        }
        dynamic.get().addInstruction(new TargetCodeInstruction(eventBodyName+"();\n"));

        generateDynamicMethodFree(bluePrint);

        bluePrint.getMethod("dynamicfree").get().addInstruction(new TargetCodeInstruction(eventBodyNameFree+"();\n"));

//        bluePrint.getMethod("dynamicfree").get().addInstruction(new TargetCodeInstruction("// free of "+eventBodyName+"\n"));

    }

    protected static void generateDynamicMethodFree(EMAMBluePrintCPP bluePrint){
        if(!bluePrint.getMethod("dynamicfree").isPresent()){
            Method f = new Method();
            f.setPublic(false);
            f.setName("dynamicfree");
            f.setReturnTypeName("void");
            bluePrint.addMethod(f);
        }
    }

    protected static List<String> getNamesOfPortsWhichAreNotInEventCondition(EMADynamicEventHandlerInstanceSymbol event, List<String> conditionNames){
        Set<String> names = new HashSet<>();
        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){
            String name = null;
            if(connector.isDynamicSourceNewPort()) {
                name = EMAPortSymbol.getNameWithoutArrayBracketPart(connector.getSourcePortName());
                if (!connector.getSourceComponentName().isPresent()) {
                    if (!conditionNames.contains(name)) {
                        names.add(name);
                    }
                }
            }

            if(connector.isDynamicTargetNewPort()){
                name = EMAPortSymbol.getNameWithoutArrayBracketPart(connector.getTargetPortName());
                if (!connector.getTargetComponentName().isPresent()) {
                    if (!conditionNames.contains(name)) {
                        names.add(name);
                    }
                }
            }
        }

        return new ArrayList<>(names);
    }

    protected static List<String> getNewInstances(EMADynamicEventHandlerInstanceSymbol event){
        Set<String> insts = new HashSet<>();
        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){
            Optional<String> comp = connector.getSourceComponentName();
            if(connector.isDynamicSourceNewComponent() && comp.isPresent()){
                insts.add(EMAPortSymbol.getNameWithoutArrayBracketPart(comp.get()));
            }

            comp = connector.getTargetComponentName();
            if(connector.isDynamicTargetNewComponent() && comp.isPresent()){
                insts.add(EMAPortSymbol.getNameWithoutArrayBracketPart(comp.get()));
            }
        }
        return new ArrayList<>(insts);
    }

    protected static Map<String, Map<String, Optional<EMADynamicConnectorInstanceSymbol>>> getNewPortsOfInstances(EMADynamicEventHandlerInstanceSymbol event, List<String> names){
//        Map<String,List<String>> newPorts = new HashMap<>();
        Map<String, Map<String, Optional<EMADynamicConnectorInstanceSymbol>>> newPorts = new HashMap<>();
        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){
            Optional<String> comp = connector.getSourceComponentName();
            String p;
            if( connector.isDynamicSourceNewPort() && comp.isPresent()){
                if(!newPorts.containsKey(comp.get())){
//                    newPorts.put(comp.get(), new ArrayList<>());
                    newPorts.put(comp.get(), new HashMap<>());
                }
                p = EMAPortSymbol.getNameWithoutArrayBracketPart(connector.getSourcePortName());
//                if(!newPorts.get(comp.get()).contains(p)){
//                    newPorts.get(comp.get()).add( p );
//                }
                if((!names.contains(comp.get()+"."+p)) && (!newPorts.get(comp.get()).containsKey(p))){
                    newPorts.get(comp.get()).put(p, Optional.empty());
                }
            }

            comp = connector.getTargetComponentName();
            if(connector.isDynamicTargetNewPort() && comp.isPresent()){
                if(!newPorts.containsKey(comp.get())){
//                    newPorts.put(comp.get(), new ArrayList<>());
                    newPorts.put(comp.get(), new HashMap<>());
                }
                p = EMAPortSymbol.getNameWithoutArrayBracketPart(connector.getTargetPortName());
//                if(!newPorts.get(comp.get()).contains(p)){
//                    newPorts.get(comp.get()).add(p);
//                }
                if(!newPorts.get(comp.get()).containsKey(p)){
                    newPorts.get(comp.get()).put(p, Optional.of(connector));
                }
            }
        }
        return newPorts;
    }

    protected static void generateHandleConnectRequestsOfQueues(List<String> names, Method body, Method free, EMADynamicEventHandlerInstanceSymbol event){
        List<String> fCondList = new ArrayList<>();
        for(String name : names){

            if(name.contains(".")){

                String c = name.substring(0, name.indexOf("."));
                String p = name.substring(name.indexOf(".")+1, name.length());

                body.addInstruction(new TargetCodeInstruction(String.format(DYNPORTIDININSTANCEGETTER, c,p,c,p, free_method_index_counter, c,p)));

                free.addInstruction(new TargetCodeInstruction(String.format(MFREE_PORTIDININSTANCE, c,p,c,p)));

                fCondList.add(String.format(MFREE_PORTIDININSTANCE_COND, c,p, free_method_index_counter));
            }else {

                body.addInstruction(new TargetCodeInstruction(
                        String.format(THISCOMPONENTPORTREQUEST,
                                name, name, name, free_method_index_counter, name)));

//                free.addInstruction(new TargetCodeInstruction(String.format(FREE_THISCOMPONENTPORTREQUEST,
//                        name, free_method_index_counter
//                )));

                free.addInstruction(new TargetCodeInstruction(String.format(MFREE_PORTID, name, name)));

                fCondList.add(String.format(MFREE_PORTID_COND, name, free_method_index_counter));
//                resetConnectedArray.add(String.format(FREE_THISCOMPONENTPORTREQUEST_CONNECTED_ARRAY, name, name));
            }
            free_method_index_counter++;
        }


        // TODO: Free stuff
        free.addInstruction(new TargetCodeInstruction("for(long i = __event_connects_"+convertName(event.getName())+".size()-1; i >= 0; --i){\n"));
        free.addInstruction(new TargetCodeInstruction("int* _connected_idxs = __event_connects_"+convertName(event.getName())+".at(i);\n"));
        free.addInstruction(new TargetCodeInstruction("if( "+String.join(" && ", fCondList)+" ){\n"));
    }

    protected static void generateHandleConnectRequestsOfThisPorts(List<String> names, Method body, Method free,EMAComponentInstanceSymbol componentSymbol){
        for(String name : names){

            int i = 1;
            while(componentSymbol.getPortInstance(name+"["+i+"]").isPresent()){
                ++i;
            }
            --i;

            body.addInstruction(new TargetCodeInstruction(
                    String.format(THISCOMPONENTPORTREQUEST_2,
                            name, i, name, name, free_method_index_counter, name)));

//            free.addInstruction(new TargetCodeInstruction(String.format(FREE_THISCOMPONENTPORTREQUEST,
//                    name, free_method_index_counter
//            )));

            free.addInstruction(new TargetCodeInstruction(String.format(MFREE_PORTID_FROM_ARRAY, name, free_method_index_counter, name, name)));


            resetConnectedArray.add(String.format(FREE_THISCOMPONENTPORTREQUEST_CONNECTED_ARRAY, name,name));

            free_method_index_counter++;
        }
        //TODO: check if names > 0
        if(names.size() > 0) {
            String c = "";
            for(int i = 0; i < free_method_index_counter; ++i){
                c += "(_connected_idxs["+i+"] < 0)";
                if( i < free_method_index_counter -1){
                    c += " || ";
                }
            }
            body.addInstruction(new TargetCodeInstruction("if("+c+"){free(_connected_idxs); return;}\n"));
            body.addInstruction(new TargetCodeInstruction("if(__parent != NULL){__parent_dynamic(__parent, true, false);}\n"));
            free.addInstruction(new TargetCodeInstruction("if(__parent != NULL){__parent_dynamic(__parent, false, true);}\n"));
        }
        for(String name : names){
            body.addInstruction(new TargetCodeInstruction(String.format(THISCOMPONENTPORTREQUEST_2_2, name, name, name)));
            free.addInstruction(new TargetCodeInstruction(String.format(MFREE_PORTID_FROM_ARRAY_2, name, name, name)));
        }

    }

    protected static void generateHandleConnectRequestsOfDynamicInstances(List<String> instancenames, Method body, EMAComponentInstanceSymbol componentSymbol, Method free){
        for (String inst : instancenames){
            int count = 1;
            while (componentSymbol.getSubComponent(inst+"["+count+"]").isPresent()){
                ++count;
            }
            body.addInstruction(new TargetCodeInstruction(String.format(DYNINSTANCECONNECT,
                    inst, count-1, inst, inst, free_method_index_counter, inst
                    )));

//            free.addInstruction(new TargetCodeInstruction(String.format(FREE_DYNINSTANCECONNECT,
//                    inst, free_method_index_counter
//            )));
            free.addInstruction(new TargetCodeInstruction(String.format(MFREE_DYNINSTID, inst, free_method_index_counter)));

            resetConnectedArray.add(String.format(FREE_DYNINSTANCECONNECT_CONNECTED_ARRAY, inst,inst));

            free_method_index_counter++;
        }
    }

    protected static void generateHandleConnectRequestInInstances(Map<String, Map<String, Optional<EMADynamicConnectorInstanceSymbol>>>  newPorts, Method body, Method free){
        for (Map.Entry<String, Map<String, Optional<EMADynamicConnectorInstanceSymbol>>> entry : newPorts.entrySet()){

            if(entry.getValue().isEmpty()){
                continue;
            }

            List<String> portList = new ArrayList<>();
            portList.addAll(entry.getValue().keySet());
            Collections.sort(portList);

            String inst = entry.getKey()+".connect_"+String.join("_", portList)+"(";
            String freeInst = entry.getKey()+".free_"+String.join("_", portList)+"(";

            String connectIdxs = "";
//            for(String port : entry.getValue()){
            for(int i = 0; i < portList.size(); ++i){

                body.addInstruction(new TargetCodeInstruction(String.format(
                        DYNPORTIDININSTANCEINIT, convertName(entry.getKey()), portList.get(i)
                )));

                inst = inst + "&"+String.format(DYNPORTIDININSTANCE, convertName(entry.getKey()), portList.get(i));

                if(entry.getValue().get(portList.get(i)).isPresent()){
                    inst += ", "+generateConnectsSource(entry.getValue().get(portList.get(i)).get());
                }

                if(i < portList.size()-1){
                    inst += ", ";
                }

//                free.addInstruction(new TargetCodeInstruction(String.format(FREE_DYNPORTIDININSTANCE,
//                        convertName(entry.getKey()), portList.get(i), free_method_index_counter)));

                free.addInstruction(new TargetCodeInstruction(String.format(MFREE_PORTIDININSTANCE_2, entry.getKey(), portList.get(i), free_method_index_counter)));


                freeInst += String.format(DYNPORTIDININSTANCE, convertName(entry.getKey()), portList.get(i));
                if(i < portList.size()-1){
                    freeInst += ", ";
                }

                connectIdxs += "_connected_idxs["+free_method_index_counter+"] = "+String.format(DYNPORTIDININSTANCE, convertName(entry.getKey()), portList.get(i))+";";

                free_method_index_counter++;


            }

            free.addInstruction(new TargetCodeInstruction(
                freeInst+");\n"
            ));

            body.addInstruction(new TargetCodeInstruction(String.format(
                    "if(!"+inst+")){ free(_connected_idxs); return ;}\n"
            )));
            body.addInstruction(new TargetCodeInstruction(connectIdxs+"\n"));

        }
    }

    protected static void generateConnects(EMADynamicEventHandlerInstanceSymbol event, Method body, EMAMBluePrintCPP bluePrint, Method executeMethod, Method free){
        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){

            Optional<String> before = generateConnectsBefore(connector);
            String sourceName = generateConnectsSource(connector);
            String targetName = generateConnectsTarget(connector);
            EMAPortInstanceSymbol target = connector.getTargetPort();

            Optional<VariableType> vt = TypeConverter.getVariableTypeForMontiCarTypeName(target.getTypeReference().getName());
            generateEventDynamicConnectVector(vt.get().getTypeNameTargetLanguage(), bluePrint);

            executeMethod.addInstruction(new ExecuteDynamicConnects(before));

            if(!before.isPresent()){
                before = Optional.of("NULL");
            }else{
                before = Optional.of("&"+before.get());
            }
            body.addInstruction(new TargetCodeInstruction(String.format(
                    "__dynamic_%s_connect.push_back({%s, &(%s), &(%s)});\n", vt.get().getTypeNameTargetLanguage(), before.get(), sourceName, targetName
            )));

            free.addInstruction(new TargetCodeInstruction(String.format("dynamicconnect_remove(&__dynamic_%s_connect, %s, &(%s), &(%s));\n",
                    vt.get().getTypeNameTargetLanguage(), before.get(), sourceName, targetName
            )));
        }
    }

    protected static String generateConnectsSource(EMADynamicConnectorInstanceSymbol connector){
        String sourceName = connector.getSource();
        if(connector.isDynamicSourceNewPort()) {
            sourceName = generateConnectNameForNewPort(sourceName, connector.getSourceComponentName(), connector.getSourcePortName());
        }else if(connector.isDynamicSourceNewComponent()){
            sourceName = generateConnectNameForNewInstance(connector.getSourceComponentName().get(), connector.getSourcePortName());
        }else{
            sourceName = GeneralHelperMethods.getTargetLanguageVariableInstanceName(sourceName);
        }
        return sourceName;
    }

    protected static String generateConnectsTarget(EMADynamicConnectorInstanceSymbol connector){
        String targetName = connector.getTarget();
        if(connector.isDynamicTargetNewPort()) {
            targetName = generateConnectNameForNewPort(targetName, connector.getTargetComponentName(), connector.getTargetPortName());
//            before = connector.getTargetComponentName();
//            if (before.isPresent()) {
//                before = Optional.of(GeneralHelperMethods.getTargetLanguageVariableInstanceName(before.get()));
//            }
        }else if(connector.isDynamicTargetNewComponent()){
            targetName = generateConnectNameForNewInstance(connector.getTargetComponentName().get(), connector.getTargetPortName());
//            String inst = EMAPortSymbol.getNameWithoutArrayBracketPart(connector.getTargetComponentName().get());
//            before = Optional.of(String.format("%s["+DYNINSTANCEID+"]", inst, inst));
        }else{
            targetName = GeneralHelperMethods.getTargetLanguageVariableInstanceName(targetName);
//            before = connector.getTargetComponentName();
//            if(before.isPresent()){
//                before = Optional.of(GeneralHelperMethods.getTargetLanguageVariableInstanceName(before.get()));
//            }
        }
        return targetName;
    }

    protected static Optional<String> generateConnectsBefore(EMADynamicConnectorInstanceSymbol connector){
        Optional<String> before = Optional.empty();
//        String targetName = connector.getTarget();
        if(connector.isDynamicTargetNewPort()) {
//            targetName = generateConnectNameForNewPort(targetName, connector.getTargetComponentName(), connector.getTargetPortName());
            before = connector.getTargetComponentName();
            if (before.isPresent()) {
                before = Optional.of(GeneralHelperMethods.getTargetLanguageVariableInstanceName(before.get()));
            }
        }else if(connector.isDynamicTargetNewComponent()){
//            targetName = generateConnectNameForNewInstance(connector.getTargetComponentName().get(), connector.getTargetPortName());
            String inst = EMAPortSymbol.getNameWithoutArrayBracketPart(connector.getTargetComponentName().get());
            before = Optional.of(String.format("%s["+DYNINSTANCEID+"]", inst, inst));
        }else{
//            targetName = GeneralHelperMethods.getTargetLanguageVariableInstanceName(targetName);
            before = connector.getTargetComponentName();
            if(before.isPresent()){
                before = Optional.of(GeneralHelperMethods.getTargetLanguageVariableInstanceName(before.get()));
            }
        }
        return before;
    }

    protected static void generateEventDynamicConnectVector(String typeName, EMAMBluePrint bluePrint){

        Optional<Method> execDynConnects = bluePrint.getMethod("executeDynamicConnects");
        if(!execDynConnects.isPresent()){
            Method m = new Method("executeDynamicConnects", "void");
            Variable p = new Variable();
            p.setName("beforeComponent");
            p.setTypeNameTargetLanguage("void*");
            m.addParameter(p);

            bluePrint.addMethod(m);
            execDynConnects = Optional.of(m);
        }

        String cConnectVarName = "__dynamic_"+typeName+"_connect";
        if(!bluePrint.getVariable(cConnectVarName).isPresent()){
            Variable vdConnect = new Variable();
            vdConnect.setName(cConnectVarName);
            vdConnect.setTypeNameTargetLanguage("std::vector<connection<"+typeName+">>");
            vdConnect.setPublic(false);
            bluePrint.addVariable(vdConnect);

            execDynConnects.get().addInstruction(new TargetCodeInstruction(String.format(
                    "for (std::vector<connection<%s>>::iterator it = __dynamic_%s_connect.begin(); it < __dynamic_%s_connect.end(); ++it) {if(it->beforeComponent == beforeComponent){*(*it).target = *(*it).source;}}\n",
                    typeName, typeName,typeName
            )));
        }
    }

    protected static String generateConnectNameForNewPort(String allName, Optional<String> componenName, String portName){
        String result = "";
        if(componenName.isPresent()){
            result = String.format("%s["+DYNPORTIDININSTANCE+"]",
                    GeneralHelperMethods.getTargetLanguageVariableInstanceName(EMAPortSymbol.getNameWithoutArrayBracketPart(allName)),
                    convertName(componenName.get()),
                    EMAPortSymbol.getNameWithoutArrayBracketPart(portName));
        }else{
            result = EMAPortSymbol.getNameWithoutArrayBracketPart(allName);
            result = String.format("%s["+DYNPORTID+"]", result, result);
        }
        return result;
    }

    protected static String generateConnectNameForNewInstance(String componentName, String portName){

        String inst = EMAPortSymbol.getNameWithoutArrayBracketPart(componentName);

        return String.format("%s["+DYNINSTANCEID+"].%s", inst, inst, GeneralHelperMethods.getTargetLanguageVariableInstanceName(portName));

    }

    protected static void generateConnectsForFreeEventBody(EMADynamicEventHandlerInstanceSymbol event, Method free){

        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){
            String sourceName = generateConnectsSource(connector);
            String targetName = generateConnectsTarget(connector);
            free.addInstruction(new TargetCodeInstruction(targetName+" = "+sourceName+";\n"));
        }
    }

    protected static String convertName(String name){
        return name.replace("[", "_").replace("]", "_");
    }

    protected static Collection<EMADynamicEventHandlerInstanceSymbol> getFreeEvents(EMAComponentInstanceSymbol component){
        if(component instanceof EMADynamicComponentInstanceSymbol) {
            return ((EMADynamicComponentInstanceSymbol)component).getEventHandlers().stream().filter(e -> e.isDynamicPortFreeEvent()).collect(Collectors.toList());
        }
        return new ArrayList<>();
    }

    //<editor-fold desc="">

    public static void resetFreePositionInCodeCounter(){
        freePositionInCodeCounter = 0;
    }

    public static boolean free_generateDynamicFreeEvent(EMADynamicEventHandlerInstanceSymbol event, EMAComponentInstanceSymbol componentSymbol, Method executeMethod, EMAMBluePrintCPP bluePrint ){
        String bodyname = "__event_body_"+event.getName().replace("[", "_").replace("]", "_");
        Method body = new Method(bodyname, "void");
        body.setPublic(false);

        generateDynamicMethodFree(bluePrint);
        List<Instruction> insts = bluePrint.getMethod("dynamicfree").get().getInstructions();
        insts.add(freePositionInCodeCounter, new TargetCodeInstruction(bodyname+"();\n"));
        bluePrint.getMethod("dynamicfree").get().setInstructions(insts);

        body.addInstruction(new TargetCodeInstruction("if(__event_condition_"+event.getName().replace("[", "_").replace("]", "_")+"()){\n"));

        List<String> names= new ArrayList<>();
        event.getCondition().getFreePortNames(names);
        java.util.Collections.sort(names);

        free_generateHandleConnectRequestsOfQueues(names, body, event);


        free_generateConnects(event, body, bluePrint, executeMethod);


        body.addInstruction(new TargetCodeInstruction("}\n"));


        freePositionInCodeCounter++;
        bluePrint.addMethod(body);
        return true;
    }

    protected static void free_generateHandleConnectRequestsOfQueues(List<String> names, Method body, EMADynamicEventHandlerInstanceSymbol event){
        for(String name : names){

            if(name.contains(".")){
                String c = name.substring(0, name.indexOf("."));
                String p = name.substring(name.indexOf(".")+1, name.length());
                body.addInstruction(new TargetCodeInstruction(String.format(EVENT_FREE_PORTIDININSTANCE, c,p,c,p)));
            }else {
                body.addInstruction(new TargetCodeInstruction(
                        String.format(EVENT_FREE_PORTID,
                                name, name)));
            }
        }
    }

    protected static void free_generateConnects(EMADynamicEventHandlerInstanceSymbol event, Method body, EMAMBluePrintCPP bluePrint, Method executeMethod){
        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){

            String sourceName = generateConnectsSource(connector);
            String targetName = generateConnectsTarget(connector);

            body.addInstruction(new TargetCodeInstruction(String.format("%s = %s;\n", targetName, sourceName)));
        }
    }
    //</editor-fold>
}
