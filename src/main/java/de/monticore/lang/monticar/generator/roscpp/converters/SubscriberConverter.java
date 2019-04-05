package de.monticore.lang.monticar.generator.roscpp.converters;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.monticar.generator.roscpp.instructions.SetPortInstruction;
import de.monticore.lang.monticar.generator.roscpp.instructions.SetStructPortInstruction;
import de.monticore.lang.monticar.generator.roscpp.util.*;

import java.util.Comparator;
import java.util.stream.Collectors;

public abstract class SubscriberConverter {
    public abstract void convertToField(RosSubscriber rosSubscriber, AdapterBluePrint bluePrint);

    public abstract void convertToInit(RosSubscriber rosSubscriber, String className, AdapterBluePrint bluePrint);

    public abstract void convertToCallback(RosSubscriber rosSubscriber, AdapterBluePrint bluePrint);

    void doConvertToCallback(RosSubscriber rosSubscriber, AdapterBluePrint bluePrint, String typeNameTargetLanguage) {
        Method method = new Method(rosSubscriber.getMethodName(), "void");
        Variable tmpParam = new Variable();
        tmpParam.setName("msg");
        tmpParam.setTypeNameTargetLanguage(typeNameTargetLanguage);
        method.addParameter(tmpParam);

        String msgField = rosSubscriber.getRosConnectionSymbol().getMsgField().orElse(null);

        EMAPortSymbol portSymbol = rosSubscriber.getPort();
        if (msgField == null) {
            method.addInstruction(new SetStructPortInstruction(portSymbol, rosSubscriber.getRosMsg()));
        } else {
            method.addInstruction(new SetPortInstruction(portSymbol, rosSubscriber.getMsgConverter()));
        }

        if (method.getInstructions().size() > 0) {
            //make instructions unique and sorted
            method.setInstructions(method.getInstructions().stream()
                    .distinct()
                    .sorted(Comparator.comparing(Instruction::getTargetLanguageInstruction))
                    .collect(Collectors.toList()));
            bluePrint.addMethod(method);
        }
    }
}
