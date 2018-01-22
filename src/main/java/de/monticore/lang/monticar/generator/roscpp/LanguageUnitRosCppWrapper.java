package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.montiarc.montiarc._symboltable.ExpandedComponentInstanceKind;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.roscpp.instructions.*;
import de.monticore.symboltable.Symbol;

import java.util.ArrayList;
import java.util.List;

public class LanguageUnitRosCppWrapper extends LanguageUnit {

    @Override
    public void generateBluePrints() {
        if (symbolsToConvert.size() > 1)
            throw new IllegalArgumentException("DataHelper does not work for multiple symbols yet!");

        for (Symbol s : symbolsToConvert) {
            if (s.isKindOf(ExpandedComponentInstanceKind.KIND)) {
                this.bluePrints.add(generateWrapperBluePrint((ExpandedComponentInstanceSymbol) s));
            }
        }

    }

    private BluePrint generateWrapperBluePrint(ExpandedComponentInstanceSymbol componentSymbol) {
        String name = componentSymbol.getFullName().replace('.', '_') + "_RosWrapper";
        BluePrint currentBluePrint = new BluePrint(name);

        generateFields(componentSymbol, currentBluePrint);
        generateCallbacks(currentBluePrint);
        generateConstructor(name, currentBluePrint);
        generatePublishMethods(currentBluePrint);
        generateTick(currentBluePrint);

        return currentBluePrint;
    }

    private void generatePublishMethods(BluePrint currentBluePrint) {
        DataHelper.getTopics().stream().filter(t -> !t.getOutgoingPorts().isEmpty())
                .forEach(t -> {
                    Method method = new Method("publish" + t.getTargetLanguageName(), "void");
                    method.addInstruction(new CreateTmpMsgInstruction(t));

                    t.getOutgoingPorts().forEach(
                            p -> method.addInstruction(new SetMsgFieldInstruction(p, DataHelper.getMsgFieldFromPort(p).orElse(null))));
                    method.addInstruction(new PublishInstruction(t));

                    t.setPublishMethod(method);


                    currentBluePrint.addMethod(method);

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

        List<Instruction> subInstructions = new ArrayList<>();
        List<Instruction> pubInstructions = new ArrayList<>();

        for (PortSymbol portSymbol : DataHelper.getPorts()) {
            if (portSymbol.isIncoming()) {
                Instruction tmpInstruction = new SubscribeInstruction(classname, DataHelper.getTopicFromPort(portSymbol).orElse(null));
                if (!subInstructions.contains(tmpInstruction)) subInstructions.add(tmpInstruction);
            } else {
                Variable publisher = DataHelper.getTopicFromPort(portSymbol).orElse(null).getPublisher().orElse(null);
                Instruction tmpInstruction = new AdvertiseInstruction(publisher, DataHelper.getTopicFromPort(portSymbol).orElse(null));
                if (!pubInstructions.contains(tmpInstruction)) pubInstructions.add(tmpInstruction);
            }
        }
        pubInstructions.forEach(constructorMethod::addInstruction);
        subInstructions.forEach(constructorMethod::addInstruction);

        currentBluePrint.addMethod(constructorMethod);
    }

    private void generateFields(ExpandedComponentInstanceSymbol symbol, BluePrint currBluePrint) {
        //component
        Variable componentField = new Variable();
        componentField.setName("component");
        //TODO: get from generator
        componentField.setTypeNameTargetLanguage(symbol.getFullName().replace(".", "_"));
        currBluePrint.addVariable(componentField);

        //subscribers and publishers

        for (RosTopic rosTopic : DataHelper.getTopics()) {
            for (PortSymbol portSymbol : rosTopic.getPorts()) {
                if (portSymbol.isIncoming()) {
                    //Generate Subscriber once per topic if needed
                    Variable field = new Variable();
                    field.setTypeNameTargetLanguage("ros::Subscriber");
                    field.setName(rosTopic.getTargetLanguageName().toLowerCase() + "Subscriber");
                    rosTopic.setSubscriber(field);
                    currBluePrint.addVariable(field);
                    break;
                }
            }

            for (PortSymbol portSymbol : rosTopic.getPorts()) {
                if (portSymbol.isOutgoing()) {
                    //Generate Publisher once per topic if needed
                    Variable field = new Variable();
                    field.setTypeNameTargetLanguage("ros::Publisher");
                    field.setName(rosTopic.getTargetLanguageName().toLowerCase() + "Publisher");
                    rosTopic.setPublisher(field);
                    currBluePrint.addVariable(field);
                    break;
                }
            }

        }

    }

    private void generateTick(BluePrint currentBluePrint) {
        Method tickMethod = new Method("tick", "void");
        tickMethod.addInstruction(new ExecuteComponentInstruction());

        DataHelper.getTopics().stream().filter(t -> t.getPublisher().isPresent())
                .forEach(t -> {
                    tickMethod.addInstruction(new CallPublishInstruction(t.getPublishMethod().orElse(null)));
                });

        currentBluePrint.addMethod(tickMethod);
    }

    private void generateCallbacks(BluePrint currentBluePrint) {

        for (RosTopic rosTopic : DataHelper.getTopics()) {
            Method method = new Method(rosTopic.getTargetLanguageName() + "Callback", "void");
            Variable tmpParam = new Variable();
            tmpParam.setName("msg");
            tmpParam.setTypeNameTargetLanguage("const " + rosTopic.getFullRosType() + "::ConstPtr&");
            method.addParameter(tmpParam);

            rosTopic.getPorts().stream()
                    .filter(PortSymbol::isIncoming)
                    .forEachOrdered(portSymbol -> {
                            method.addInstruction(new SetPortInstruction(portSymbol, DataHelper.getMsgFieldFromPort(portSymbol).orElse(null)));
                    });

            if (method.getInstructions().size() > 0) {
                rosTopic.setCallback(method);
                currentBluePrint.addMethod(method);
            }

        }

    }

}
