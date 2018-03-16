package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.roscpp.instructions.*;
import de.monticore.lang.monticar.generator.rosmsg.GeneratorRosMsg;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;

import java.util.*;
import java.util.stream.Collectors;

public class LanguageUnitRosCppAdapter {

    private List<BluePrintCPP> bluePrints = new ArrayList<>();
    private Map<Variable, RosConnectionSymbol> subscribers = new HashMap<>();
    private Map<Variable, RosConnectionSymbol> publishers = new HashMap<>();
    private List<Method> publishMethods = new ArrayList<>();
    private List<MsgConverter> msgConverts = new ArrayList<>();
    private List<String> additionalPackages = new ArrayList<>();
    private Map<RosMsg, MCTypeReference<? extends MCTypeSymbol>> usedRosMsgs = new HashMap<>();

    public List<String> getAdditionalPackages() {
        return additionalPackages;
    }

    public void generateBluePrints(ExpandedComponentInstanceSymbol component) {
        this.bluePrints.add(generateAdapterBluePrint(component));
    }

    private BluePrintCPP generateAdapterBluePrint(ExpandedComponentInstanceSymbol componentSymbol) {

        String name = NameHelper.getAdapterName(componentSymbol);
        BluePrintCPP currentBluePrint = new BluePrintCPP(name);

        List<PortSymbol> rosPorts = componentSymbol.getPorts().stream()
                .filter(PortSymbol::isRosPort)
                .collect(Collectors.toList());

        generateFields(componentSymbol, rosPorts, currentBluePrint);
        generateCallbacks(rosPorts, currentBluePrint);
        generateConstructor(name, currentBluePrint);
        generateInit(name, NameHelper.getComponentNameTargetLanguage(componentSymbol.getFullName()), currentBluePrint);
        generatePublishMethods(rosPorts, currentBluePrint);
        generateTick(currentBluePrint);
        generateIncludes(componentSymbol, rosPorts, currentBluePrint);

        return currentBluePrint;
    }

    private void generateIncludes(ExpandedComponentInstanceSymbol componentSymbol, List<PortSymbol> rosPorts, BluePrintCPP currentBluePrint) {
        currentBluePrint.addAdditionalIncludeString("<ros/ros.h>");
        String compName = NameHelper.getComponentNameTargetLanguage(componentSymbol.getFullName());
        currentBluePrint.addAdditionalIncludeString("\"" + compName + ".h\"");
        currentBluePrint.addAdditionalIncludeString("\"IAdapter_" + compName + ".h\"");
        //Add each msg include exactly once

        rosPorts.stream()
                .map(PortSymbol::getMiddlewareSymbol)
                .map(Optional::get)
                .map(mws -> (RosConnectionSymbol) mws)
                .map(RosConnectionSymbol::getTopicType)
                .map(Optional::get)
                .peek(topicType -> additionalPackages.add(NameHelper.getPackageOfMsgType(topicType)))
                .map(type -> "<" + type + ".h>")
                .forEach(currentBluePrint::addAdditionalIncludeString);

        msgConverts.stream()
                .map(MsgConverter::getAdditionalIncludes)
                .flatMap(Collection::stream)
                .distinct()
                .forEach(currentBluePrint::addAdditionalIncludeString);
    }

    private void generatePublishMethods(List<PortSymbol> rosPorts, BluePrint currentBluePrint) {
        Map<String, Method> topicToMethod = new HashMap<>();

        publishers.keySet().forEach(var -> {
            Method method = new Method("publish" + var.getNameTargetLanguageFormat(), "void");
            method.addInstruction(new CreateTmpMsgInstruction(getFullRosType(publishers.get(var))));
            topicToMethod.put(publishers.get(var).getTopicName().get(), method);
            publishMethods.add(method);
        });

        rosPorts.stream()
                .filter(PortSymbol::isOutgoing)
                .sorted(Comparator.comparing(PortSymbol::getName))
                .forEachOrdered(p -> {
                    RosConnectionSymbol rcs = (RosConnectionSymbol) p.getMiddlewareSymbol().get();
                    Method method = topicToMethod.get(rcs.getTopicName().get());

                    if (!rcs.getMsgField().isPresent()) {
                        String packageName = Arrays.stream(rcs.getTopicType().get().split("/")).findFirst().get();
                        RosMsg rosMsg = GeneratorRosMsg.getRosType(packageName, p.getTypeReference());
                        usedRosMsgs.put(rosMsg, p.getTypeReference());
                        method.addInstruction(new SetStructMsgInstruction(p, rosMsg));
                    } else {
                        SetMsgFieldInstruction tmpInstr = new SetMsgFieldInstruction(p, getMsgConverter(rcs.getMsgField().get(), p.isIncoming()));
                        method.addInstruction(tmpInstr);
                    }
                });

        publishers.keySet().forEach(var -> {
            Method method = topicToMethod.get(publishers.get(var).getTopicName().get());
            method.addInstruction(new PublishInstruction(var));
            currentBluePrint.addMethod(method);
        });
    }

    private String getTopicNameTargetLanguage(String topicName) {
        return topicName.replace("/", "_")
                .replace("[", "_")
                .replace("]", "_");
    }

    private void generateConstructor(String classname, BluePrint currentBluePrint) {
        Method constructorMethod = new Method(classname, "");
        constructorMethod.addInstruction(new TargetCodeInstruction(""));
        currentBluePrint.addMethod(constructorMethod);
    }

    public void generateInit(String classname, String componentName, BluePrint currentBluePrint) {
        Method initMethod = new Method("init", "void");

        Variable compPointer = new Variable();
        compPointer.setTypeNameTargetLanguage(componentName + "*");
        compPointer.setName("comp");
        initMethod.addParameter(compPointer);

        initMethod.addInstruction(new TargetCodeInstruction("this->component = comp;"));
        initMethod.addInstruction(new TargetCodeInstruction("char* tmp = strdup(\"\");"));
        initMethod.addInstruction(new TargetCodeInstruction("int i = 0;"));
        initMethod.addInstruction(new TargetCodeInstruction("ros::init(i, &tmp, \"" + classname + "_node\");"));
        initMethod.addInstruction(new TargetCodeInstruction("ros::NodeHandle node_handle = ros::NodeHandle();"));

        //subs
        subscribers.keySet().stream()
                .map(var -> new SubscribeInstruction(classname, var, subscribers.get(var).getTopicName().get(), getTopicNameTargetLanguage(subscribers.get(var).getTopicName().get()) + "Callback"))
                .distinct()
                .sorted(Comparator.comparing(SubscribeInstruction::getTargetLanguageInstruction))
                .forEach(initMethod::addInstruction);

        publishers.keySet().stream()
                .map(var -> new AdvertiseInstruction(var, getFullRosType(publishers.get(var)), publishers.get(var).getTopicName().get()))
                .distinct()
                .sorted(Comparator.comparing(AdvertiseInstruction::getTargetLanguageInstruction))
                .forEach(initMethod::addInstruction);

        initMethod.addInstruction(new TargetCodeInstruction("ros::spin();"));

        currentBluePrint.addMethod(initMethod);
    }

    private void generateFields(ExpandedComponentInstanceSymbol symbol, List<PortSymbol> rosPorts, BluePrintCPP currBluePrint) {
        currBluePrint.addDefineGenerics(symbol);
        //component
        Variable componentField = new Variable();
        componentField.setName("component");
        //TODO: get from generator
        componentField.setTypeNameTargetLanguage(NameHelper.getComponentNameTargetLanguage(symbol.getFullName()) + "*");
        currBluePrint.addVariable(componentField);

        Map<String, Variable> uniqueSubFields = new HashMap<>();
        Map<String, Variable> uniquePubFields = new HashMap<>();

        //subs
        rosPorts.stream()
                .filter(PortSymbol::isIncoming)
                .map(p -> (RosConnectionSymbol) p.getMiddlewareSymbol().get())
                .forEach(rosConnectionSymbol -> {
                    String name = getTopicNameTargetLanguage(rosConnectionSymbol.getTopicName().get()).toLowerCase() + "Subscriber";
                    if (!uniqueSubFields.containsKey(name)) {
                        Variable field = new Variable();
                        field.setTypeNameTargetLanguage("ros::Subscriber");
                        field.setName(name);
                        uniqueSubFields.put(name, field);
                        currBluePrint.addVariable(field);
                        subscribers.put(field, rosConnectionSymbol);
                    }
                });

        //pubs
        rosPorts.stream()
                .filter(PortSymbol::isOutgoing)
                .map(p -> (RosConnectionSymbol) p.getMiddlewareSymbol().get())
                .forEach(rosConnectionSymbol -> {
                    String name = getTopicNameTargetLanguage(rosConnectionSymbol.getTopicName().get()).toLowerCase() + "Publisher";
                    if (!uniquePubFields.containsKey(name)) {
                        Variable field = new Variable();
                        field.setTypeNameTargetLanguage("ros::Publisher");
                        field.setName(name);
                        uniquePubFields.put(name, field);
                        currBluePrint.addVariable(field);
                        publishers.put(field, rosConnectionSymbol);
                    }
                });

    }


    private void generateTick(BluePrint currentBluePrint) {
        Method tickMethod = new Method("tick", "void");

        publishMethods.stream()
                .sorted(Comparator.comparing(Method::getName))
                .map(CallMethodInstruction::new)
                .forEachOrdered(tickMethod::addInstruction);

        if (tickMethod.getInstructions().size() == 0)
            tickMethod.addInstruction(new TargetCodeInstruction(""));

        currentBluePrint.addMethod(tickMethod);
    }


    private String getFullRosType(RosConnectionSymbol rosConnectionSymbol) {
        return rosConnectionSymbol.getTopicType().get().replace("/", "::");
    }

    private void generateCallbacks(List<PortSymbol> rosPorts, BluePrint currentBluePrint) {

        Map<String, Method> uniqueMethods = new HashMap<>();


        rosPorts.stream()
                .filter(PortSymbol::isIncoming)
                .forEachOrdered(portSymbol -> {
                    RosConnectionSymbol rosCon = (RosConnectionSymbol) portSymbol.getMiddlewareSymbol().get();
                    if (!uniqueMethods.containsKey(rosCon.getTopicName().get())) {
                        Method method = new Method(getTopicNameTargetLanguage(rosCon.getTopicName().get()) + "Callback", "void");
                        Variable tmpParam = new Variable();
                        tmpParam.setName("msg");
                        tmpParam.setTypeNameTargetLanguage("const " + getFullRosType(rosCon) + "::ConstPtr&");
                        method.addParameter(tmpParam);
                        uniqueMethods.put(rosCon.getTopicName().get(), method);
                    }

                    Method method = uniqueMethods.get(rosCon.getTopicName().get());

                    String msgField = rosCon.getMsgField().orElse(null);

                    if (msgField == null) {
                        //TODO: checks?
                        String packageName = Arrays.stream(rosCon.getTopicType().get().split("/")).findFirst().get();
                        RosMsg rosMsg = GeneratorRosMsg.getRosType(packageName, portSymbol.getTypeReference());
                        usedRosMsgs.put(rosMsg, portSymbol.getTypeReference());
                        method.addInstruction(new SetStructPortInstruction(portSymbol, rosMsg));
                    } else {
                        method.addInstruction(new SetPortInstruction(portSymbol, getMsgConverter(msgField, portSymbol.isIncoming())));
                    }

                    if (method.getInstructions().size() > 0) {
                        //make instructions unique and sorted
                        method.setInstructions(method.getInstructions().stream()
                                .distinct()
                                .sorted(Comparator.comparing(Instruction::getTargetLanguageInstruction))
                                .collect(Collectors.toList()));
                        currentBluePrint.addMethod(method);
                    }
                });

    }

    private MsgConverter getMsgConverter(String msgField, boolean incoming) {
        MsgConverter tmpMsgConverter;
        if (!msgField.contains("::")) {
            tmpMsgConverter = new DirectMsgConverter(msgField, incoming);
        } else {
            String className = msgField.substring(0, msgField.lastIndexOf("::"));
            tmpMsgConverter = new MethodMsgConverter(msgField, "\"" + className + ".h\"", incoming);
        }
        msgConverts.add(tmpMsgConverter);
        return tmpMsgConverter;
    }

    public List<BluePrintCPP> getBluePrints() {
        return bluePrints;
    }

    public Map<RosMsg, MCTypeReference<? extends MCTypeSymbol>> getUsedRosMsgs() {
        return usedRosMsgs;
    }
}
