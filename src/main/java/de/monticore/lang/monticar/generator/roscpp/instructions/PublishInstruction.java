package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.RosTopic;

public class PublishInstruction extends TargetCodeInstruction {

    public PublishInstruction(RosTopic rosTopic) {
        this.instruction = rosTopic.getPublisher().orElse(null).getNameTargetLanguageFormat() + ".publish(tmpMsg);";
    }
}
