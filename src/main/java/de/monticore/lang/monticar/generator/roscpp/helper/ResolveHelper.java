package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticar.lang.monticar.generator.python.RosInterface;
import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.roscpp.*;
import de.monticore.symboltable.Scope;

public class ResolveHelper {

    public static ResolvedRosInterface resolveRosInterface(RosInterface rosInterface, ExpandedComponentInstanceSymbol component, boolean isSubscriber) {
        String includeString = rosInterface.type;
        if (!includeString.contains("/")) {
            throw new IllegalArgumentException("The ROS msg type has to be given in the form package/msgName!");
        }
        String topicType = includeString.substring(includeString.lastIndexOf("/") + 1);
        ResolvedRosInterface res = new ResolvedRosInterface(topicType, rosInterface.topic, includeString);

        rosInterface.ports.keySet().forEach(portName -> {
            PortSymbol tmpPort = component.getPort(portName).
                    orElseThrow(() -> new RuntimeException("Port " + component.getName() + "." + portName + " not found!"));

            MsgConverter tmpMsgConverter;
            if (!rosInterface.ports.get(portName).contains("::")) {
                tmpMsgConverter = new DirectMsgConverter(rosInterface.ports.get(portName), isSubscriber);
            } else {
                String fullMethodName = rosInterface.ports.get(portName);
                String className = fullMethodName.substring(0, fullMethodName.lastIndexOf("::"));
                tmpMsgConverter = new MethodMsgConverter(fullMethodName, "\"" + className + ".h\"", isSubscriber);
            }
            res.addPort(tmpPort, tmpMsgConverter);

        });
        return res;
    }

    public static ResolvedRosTag resolveRosTag(RosTag rosTag, Scope symtab) {

        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve(rosTag.component, ExpandedComponentInstanceSymbol.KIND)
                .orElseThrow(() -> new RuntimeException("Component " + rosTag.component + " could not be found!"));

        ResolvedRosTag res = new ResolvedRosTag(componentInstanceSymbol);

        rosTag.publisher.stream().map(p -> resolveRosInterface(p, componentInstanceSymbol, false)).forEach(res::addPublisherInterface);
        rosTag.subscriber.stream().map(s -> resolveRosInterface(s, componentInstanceSymbol, true)).forEach(res::addSubscriberInterface);

        return res;
    }

}
