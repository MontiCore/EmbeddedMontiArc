package de.monticore.lang.monticar.generator.roscpp.converters;

import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.roscpp.instructions.SubscribeInstruction;
import de.monticore.lang.monticar.generator.roscpp.util.AdapterBluePrint;
import de.monticore.lang.monticar.generator.roscpp.util.Method;
import de.monticore.lang.monticar.generator.roscpp.util.RosSubscriber;
import de.monticore.lang.monticar.generator.roscpp.util.Variable;

public class RosSubscriberConverter extends SubscriberConverter {

    @Override
    public void convertToField(RosSubscriber rosSubscriber, AdapterBluePrint bluePrint) {
        Variable field = new Variable();
        field.setTypeNameTargetLanguage("ros::Subscriber");
        field.setName(rosSubscriber.getNameInTargetLanguage());
        bluePrint.addVariable(field);
    }

    @Override
    public void convertToInit(RosSubscriber rosSubscriber, String className, AdapterBluePrint bluePrint) {
        Method init = bluePrint.getInit();
        init.addInstruction(new SubscribeInstruction(className, rosSubscriber.getNameInTargetLanguage(), rosSubscriber.getRosConnectionSymbol().getTopicName().get(), rosSubscriber.getMethodName(), false, rosSubscriber.getTypeNameInTargetLanguage()));
    }

    @Override
    public void convertToCallback(RosSubscriber rosSubscriber, AdapterBluePrint bluePrint) {
        RosConnectionSymbol rosCon = rosSubscriber.getRosConnectionSymbol();
        String typeNameTargetLanguage = "const " + NameHelper.getFullRosType(rosCon) + "::ConstPtr&";
        doConvertToCallback(rosSubscriber, bluePrint, typeNameTargetLanguage);
    }


}
