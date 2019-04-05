package de.monticore.lang.monticar.generator.roscpp.converters;

import de.monticore.lang.monticar.generator.roscpp.instructions.CreateTmpMsgInstruction;
import de.monticore.lang.monticar.generator.roscpp.instructions.SetMsgFieldInstruction;
import de.monticore.lang.monticar.generator.roscpp.instructions.SetStructMsgInstruction;
import de.monticore.lang.monticar.generator.roscpp.util.AdapterBluePrint;
import de.monticore.lang.monticar.generator.roscpp.util.Method;
import de.monticore.lang.monticar.generator.roscpp.util.RosPublisher;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;

public abstract class PublisherConverter {
    public abstract void convertToField(RosPublisher rosPublisher, AdapterBluePrint bluePrint);

    public abstract void convertToPublishMethod(RosPublisher rosPublisher, AdapterBluePrint bluePrint);

    public abstract void convertToInit(RosPublisher rosPublisher, AdapterBluePrint bluePrint);

    Method doConvertToPublishMethod(RosPublisher rosPublisher, RosMsg rosMsg, AdapterBluePrint bluePrint, String fullType) {
        Method method = new Method(rosPublisher.getMethodName(), "void");
        method.addInstruction(new CreateTmpMsgInstruction(fullType));

        if (!rosPublisher.getRosConnectionSymbol().getMsgField().isPresent()) {
            method.addInstruction(new SetStructMsgInstruction(rosPublisher.getPort(), rosMsg));
        } else {
            SetMsgFieldInstruction tmpInstr = new SetMsgFieldInstruction(rosPublisher.getPort(), rosPublisher.getMsgConverter());
            method.addInstruction(tmpInstr);
        }

        bluePrint.addMethod(method);
        return method;
    }

}
