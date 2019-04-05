package de.monticore.lang.monticar.generator.roscpp.converters;

import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.roscpp.instructions.AdvertiseInstruction;
import de.monticore.lang.monticar.generator.roscpp.instructions.PublishInstruction;
import de.monticore.lang.monticar.generator.roscpp.util.AdapterBluePrint;
import de.monticore.lang.monticar.generator.roscpp.util.Method;
import de.monticore.lang.monticar.generator.roscpp.util.RosPublisher;
import de.monticore.lang.monticar.generator.roscpp.util.Variable;

public class RosPublisherConverter extends PublisherConverter {

    @Override
    public void convertToField(RosPublisher rosPublisher, AdapterBluePrint bluePrint) {
        Variable field = new Variable();
        field.setTypeNameTargetLanguage("ros::Publisher");
        field.setName(rosPublisher.getNameInTargetLanguage());
        bluePrint.addVariable(field);
    }

    @Override
    public void convertToPublishMethod(RosPublisher rosPublisher, AdapterBluePrint bluePrint) {
        String fullRosType = NameHelper.getFullRosType(rosPublisher.getRosConnectionSymbol());
        Method method = doConvertToPublishMethod(rosPublisher, rosPublisher.getRosMsg(), bluePrint, fullRosType);
        method.addInstruction(new PublishInstruction(rosPublisher.getNameInTargetLanguage(), false));
    }

    @Override
    public void convertToInit(RosPublisher rosPublisher, AdapterBluePrint bluePrint) {
        Method method = bluePrint.getInit();
        method.addInstruction(new AdvertiseInstruction(rosPublisher.getNameInTargetLanguage(), rosPublisher.getTypeNameInTargetLanguage(), rosPublisher.getRosConnectionSymbol().getTopicName().get(), false));
    }


}
