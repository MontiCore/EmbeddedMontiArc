package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.montiarc.montiarc._symboltable.ExpandedComponentInstanceKind;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.LanguageUnit;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.roscpp.instructions.*;
import de.monticore.symboltable.Symbol;

import java.util.Optional;

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

    //TODO: multiple publishers per topic is possible, not generated that way
    private void generatePublishMethods(BluePrint currentBluePrint) {
        DataHelper.getTopics().stream()
                .filter(t -> !t.getOutgoingPorts().isEmpty())
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

        //add subscribe and advertise instructions
        DataHelper.getPorts().stream()
                .filter(PortSymbol::isIncoming)
                .map(p -> new SubscribeInstruction(classname, DataHelper.getTopicFromPort(p).orElse(null)))
                .distinct()
                .forEach(constructorMethod::addInstruction);

        DataHelper.getPorts().stream()
                .filter(PortSymbol::isOutgoing)
                .map(p -> new AdvertiseInstruction(DataHelper.getTopicFromPort(p).orElse(null)))
                .distinct()
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

        //subscribers and publishers
        for (RosTopic rosTopic : DataHelper.getTopics()) {
            //Generate Subscriber once per topic if needed
            if (!rosTopic.getIncommingPorts().isEmpty()) {
                Variable field = new Variable();
                field.setTypeNameTargetLanguage("ros::Subscriber");
                field.setName(rosTopic.getTargetLanguageName().toLowerCase() + "Subscriber");
                rosTopic.setSubscriber(field);
                currBluePrint.addVariable(field);
            }

            //Generate Publisher once per topic if needed
            if (!rosTopic.getOutgoingPorts().isEmpty()) {
                Variable field = new Variable();
                field.setTypeNameTargetLanguage("ros::Publisher");
                field.setName(rosTopic.getTargetLanguageName().toLowerCase() + "Publisher");
                rosTopic.setPublisher(field);
                currBluePrint.addVariable(field);
            }
        }

    }


    private void generateTick(BluePrint currentBluePrint) {
        Method tickMethod = new Method("tick", "void");
        tickMethod.addInstruction(new ExecuteComponentInstruction());

        DataHelper.getTopics().stream()
                .map(RosTopic::getPublishMethod)
                .filter(Optional::isPresent)
                .forEach(optionalMethod -> {
                    tickMethod.addInstruction(new CallPublishInstruction(optionalMethod.get()));
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
