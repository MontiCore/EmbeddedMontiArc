package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticar.lang.monticar.generator.python.RosInterface;
import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.monticar.generator.roscpp.ResolvedRosInterface;
import de.monticore.lang.monticar.generator.roscpp.ResolvedRosTag;
import de.monticore.symboltable.Scope;

public class ResolveHelper {

    private ResolveHelper() {
    }

    //TODO: refactor
    public static ResolvedRosInterface resolveRosInterface(RosInterface rosInterface, EMAComponentInstanceSymbol component, boolean isSubscriber) {
        String includeString = rosInterface.type;
        if (!includeString.contains("/")) {
            throw new IllegalArgumentException("The ROS msg type has to be given in the form package/msgName!");
        }
        String topicType = includeString.substring(includeString.lastIndexOf("/") + 1);
        ResolvedRosInterface res = new ResolvedRosInterface(topicType, rosInterface.topic, includeString);

        rosInterface.ports.keySet().forEach(portName -> {
            EMAPortSymbol tmpPort = component.getPortInstance(portName).
                    orElseThrow(() -> new RuntimeException("Port " + component.getName() + "." + portName + " not found!"));

            res.addPort(tmpPort, rosInterface.ports.get(portName));
        });
        return res;
    }

    public static ResolvedRosTag resolveRosTag(RosTag rosTag, Scope symtab) {

        EMAComponentInstanceSymbol componentInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve(rosTag.component, EMAComponentInstanceSymbol.KIND)
                .orElseThrow(() -> new RuntimeException("Component " + rosTag.component + " could not be found!"));

        ResolvedRosTag res = new ResolvedRosTag(componentInstanceSymbol);

        rosTag.publisher.stream().map(p -> resolveRosInterface(p, componentInstanceSymbol, false)).forEach(res::addPublisherInterface);
        rosTag.subscriber.stream().map(s -> resolveRosInterface(s, componentInstanceSymbol, true)).forEach(res::addSubscriberInterface);

        return res;
    }

}
