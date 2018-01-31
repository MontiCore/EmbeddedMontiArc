package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.roscpp.instructions.*;

import java.util.*;
import java.util.stream.Collectors;

public class LanguageUnitRosCppWrapper {

    
    
    private ResolvedRosTag resolvedRosTag;
    private List<BluePrintCPP> bluePrints = new ArrayList<>();

    public void setResolvedRosTag(ResolvedRosTag resolvedRosTag) {
        this.resolvedRosTag = resolvedRosTag;
    }

    public void generateBluePrints(ResolvedRosTag resolvedRosTag) {
        if (resolvedRosTag != null && resolvedRosTag.getComponent() != null) {
            this.setResolvedRosTag(resolvedRosTag);
            this.bluePrints.add(generateWrapperBluePrint(resolvedRosTag.getComponent()));
        } else {
            throw new IllegalArgumentException("resolvedRosTag and resolvedRosTag.component must not be null!");
        }
    }

    private BluePrintCPP generateWrapperBluePrint(ExpandedComponentInstanceSymbol componentSymbol) {

        String name = componentSymbol.getFullName().replace('.', '_') + "_RosWrapper";
        BluePrintCPP currentBluePrint = new BluePrintCPP(name);

        generateFields(componentSymbol, currentBluePrint);
        generateCallbacks(currentBluePrint);
        generateConstructor(name, currentBluePrint);
        generatePublishMethods(currentBluePrint);
        generateTick(currentBluePrint);
        generateIncludes(currentBluePrint);

        return currentBluePrint;
    }

    private void generateIncludes(BluePrintCPP currentBluePrint) {
        currentBluePrint.addAdditionalIncludeString("<ros/ros.h>");
        ExpandedComponentInstanceSymbol componentSymbol = resolvedRosTag.getComponent();
        currentBluePrint.addAdditionalIncludeString("\"" + componentSymbol.getFullName().replace(".", "_") + ".h\"");
        //Add each msg include exactly once
        Set<ResolvedRosInterface> allInterfaces = new HashSet<>();
        allInterfaces.addAll(resolvedRosTag.getPublisherInterfaces());
        allInterfaces.addAll(resolvedRosTag.getSubscriberInterfaces());

        allInterfaces.stream()
                .map(t -> "<" + t.getInclude() + ".h>")
                .distinct()
                .sorted()
                .forEach(currentBluePrint::addAdditionalIncludeString);

        allInterfaces.stream()
                .flatMap(i -> i.getMsgConverters().stream())
                .flatMap(conv -> conv.getAdditionalIncludes().stream())
                .distinct()
                .sorted()
                .forEach(currentBluePrint::addAdditionalIncludeString);


    }

    private void generatePublishMethods(BluePrint currentBluePrint) {
        final int[] i = {0};
        resolvedRosTag.getPublisherInterfaces().stream()
                .sorted(Comparator.comparing(ResolvedRosInterface::getTargetLanguageName))
                .forEach(pubInter -> {
                    Method method = new Method("publish" + i[0], "void");
                    method.addInstruction(new CreateTmpMsgInstruction(pubInter.getFullRosType()));

                    pubInter.getPorts().stream()
                            .sorted(Comparator.comparing(PortSymbol::getName))
                            .forEach(
                            p -> method.addInstruction(new SetMsgFieldInstruction(p, pubInter.getMsgFieldForPort(p))));

                    method.addInstruction(new PublishInstruction(pubInter.getPublisherField().get()));
                    pubInter.setPublishMethod(method);
                    currentBluePrint.addMethod(method);
                    i[0]++;
                });

    }

    private void generateConstructor(String classname, BluePrint currentBluePrint) {
        Method constructorMethod = new Method(classname, "");
        Variable param1 = new Variable();
        param1.setName("node_handle");
        param1.setTypeNameTargetLanguage("ros::NodeHandle");

        Variable param2 = new Variable();
        param2.setName("private_node_handle");
        param2.setTypeNameTargetLanguage("ros::NodeHandle");

        constructorMethod.addParameter(param1);
        constructorMethod.addParameter(param2);

        resolvedRosTag.getSubscriberInterfaces().stream()
                .filter(inter -> inter.getSubscriberField().isPresent())
                .map(inter -> new SubscribeInstruction(classname, inter.getSubscriberField().get(), inter.getTopic(), inter.getTargetLanguageName() + "Callback"))
                .distinct()
                .sorted(Comparator.comparing(SubscribeInstruction::getTargetLanguageInstruction))
                .forEach(constructorMethod::addInstruction);

        resolvedRosTag.getPublisherInterfaces().stream()
                .filter(inter -> inter.getPublisherField().isPresent())
                .map(inter -> new AdvertiseInstruction(inter.getPublisherField().get(), inter.getFullRosType(), inter.getTopic()))
                .distinct()
                .sorted(Comparator.comparing(AdvertiseInstruction::getTargetLanguageInstruction))
                .forEach(constructorMethod::addInstruction);

        currentBluePrint.addMethod(constructorMethod);
    }

    private void generateFields(ExpandedComponentInstanceSymbol symbol, BluePrint currBluePrint) {
        //component
        Variable componentField = new Variable();
        componentField.setName("component");
        //TODO: get from generator
        componentField.setTypeNameTargetLanguage(symbol.getFullName().replace(".", "_"));
        currBluePrint.addVariable(componentField);

        Map<String, Variable> uniqueSubFields = new HashMap<>();
        Map<String, Variable> uniquePubFields = new HashMap<>();

        resolvedRosTag.getSubscriberInterfaces()
                .forEach(subInter -> {
                    String name = subInter.getTargetLanguageName().toLowerCase() + "Subscriber";
                    if (!uniqueSubFields.containsKey(name)) {
                        Variable field = new Variable();
                        field.setTypeNameTargetLanguage("ros::Subscriber");
                        field.setName(name);
                        uniqueSubFields.put(name, field);
                    }
                    subInter.setSubscriberField(uniqueSubFields.get(name));
                });

        resolvedRosTag.getPublisherInterfaces()
                .forEach(pubInter -> {
                    String name = pubInter.getTargetLanguageName().toLowerCase() + "Publisher";
                    if (!uniquePubFields.containsKey(name)) {
                        Variable field = new Variable();
                        field.setTypeNameTargetLanguage("ros::Publisher");
                        field.setName(name);
                        uniquePubFields.put(name, field);
                    }
                    pubInter.setPublisherField(uniquePubFields.get(name));
                });

        uniqueSubFields.values().forEach(currBluePrint::addVariable);
        uniquePubFields.values().forEach(currBluePrint::addVariable);
    }


    private void generateTick(BluePrint currentBluePrint) {
        Method tickMethod = new Method("tick", "void");
        tickMethod.addInstruction(new ExecuteComponentInstruction());

        resolvedRosTag.getPublisherInterfaces().stream()
                .map(ResolvedRosInterface::getPublishMethod)
                .map(Optional::get)
                .sorted(Comparator.comparing(Method::getName))
                .forEach(publishMethod -> tickMethod.addInstruction(new CallMethodInstruction(publishMethod)));

        currentBluePrint.addMethod(tickMethod);
    }

    private void generateCallbacks(BluePrint currentBluePrint) {

        Map<String, Method> uniqueMethods = new HashMap<>();

        resolvedRosTag.getSubscriberInterfaces().forEach(subInter -> {
            if (!uniqueMethods.containsKey(subInter.getTargetLanguageName())) {
                Method method = new Method(subInter.getTargetLanguageName() + "Callback", "void");
                Variable tmpParam = new Variable();
                tmpParam.setName("msg");
                tmpParam.setTypeNameTargetLanguage("const " + subInter.getFullRosType() + "::ConstPtr&");
                method.addParameter(tmpParam);
                uniqueMethods.put(subInter.getTargetLanguageName(), method);
            }

            Method method = uniqueMethods.get(subInter.getTargetLanguageName());

            subInter.getPorts().stream()
                    .filter(PortSymbol::isIncoming)
                    .forEachOrdered(portSymbol -> {
                        method.addInstruction(new SetPortInstruction(portSymbol, subInter.getMsgFieldForPort(portSymbol)));
                    });

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

    public List<BluePrintCPP> getBluePrints() {
        return bluePrints;
    }
}
