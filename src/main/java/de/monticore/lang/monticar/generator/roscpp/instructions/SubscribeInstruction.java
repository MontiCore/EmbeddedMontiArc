package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.roscpp.RosTopic;

public class SubscribeInstruction implements Instruction {
    private static final int MSG_QUEUE_SIZE = 5;
    private String targetLanguageInstruction;

    public SubscribeInstruction(String className, PortSymbol portSymbol, RosTopic rosTopic) {
        String topicName = rosTopic.getName();
        String rosType = rosTopic.getRosType();
        String callback = rosTopic.getCallback().orElse(null).getName();
        Variable subscriber = rosTopic.getSubscriber().orElse(null);

        targetLanguageInstruction = subscriber.getNameTargetLanguageFormat() + " = node_handle.subscribe(\"" + topicName + "\" ," + MSG_QUEUE_SIZE + ",&" + className + "::" + callback + ", this, ros::TransportHints().tcpNoDelay());";
//        targetLanguageInstruction = subscriber.getNameTargetLanguageFormat() + " = node_handle.subscribe(\"" + topicName + "\" ," + MSG_QUEUE_SIZE + ",&" + className + "::" + rosTopic.getTargetLanguageName() + "Callback" + ", this, ros::TransportHints().tcpNoDelay());";
    }

    @Override
    public String getTargetLanguageInstruction() {
        return targetLanguageInstruction;
    }

    @Override
    public boolean isConnectInstruction() {
        return false;
    }

    @Override
    public boolean isTargetCodeInstruction() {
        return true;
    }

    @Override
    public boolean isExecuteInstruction() {
        return false;
    }

    @Override
    public boolean equals(Object other) {
        if (!(other instanceof SubscribeInstruction)) return false;

        return getTargetLanguageInstruction().equals(((SubscribeInstruction) other).getTargetLanguageInstruction());
    }


}
