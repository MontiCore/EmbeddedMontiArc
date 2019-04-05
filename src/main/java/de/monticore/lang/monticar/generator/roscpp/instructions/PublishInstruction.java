package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;

public class PublishInstruction extends TargetCodeInstruction {

    public PublishInstruction(String publisherFieldName, boolean ros2Mode) {
        if(!ros2Mode) {
            this.instruction = publisherFieldName + ".publish(tmpMsg);";
        }else{
            this.instruction = publisherFieldName + "->publish(tmpMsg);";
        }
    }
}
