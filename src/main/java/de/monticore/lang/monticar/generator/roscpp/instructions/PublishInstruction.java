package de.monticore.lang.monticar.generator.roscpp.instructions;

import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.roscpp.util.Variable;

public class PublishInstruction extends TargetCodeInstruction {

    public PublishInstruction(Variable publisher) {
        this.instruction = publisher.getNameTargetLanguageFormat() + ".publish(tmpMsg);";
    }
}
