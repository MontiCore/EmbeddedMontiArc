package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.converters.*;
import de.monticore.lang.monticar.generator.roscpp.helper.InitHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.monticar.generator.roscpp.instructions.CallMethodInstruction;
import de.monticore.lang.monticar.generator.roscpp.util.*;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;

import java.util.*;
import java.util.stream.Collectors;

public class LanguageUnitRosCppAdapter {

    private List<DirectMsgConverter> msgConverts = new ArrayList<>();
    private List<String> additionalPackages = new ArrayList<>();
    private Map<RosMsg, MCTypeReference<? extends MCTypeSymbol>> usedRosMsgs = new HashMap<>();
    private boolean ros2Mode = false;
    private List<RosInterface> interfaces;

    public LanguageUnitRosCppAdapter(boolean ros2Mode) {
        this.ros2Mode = ros2Mode;
    }

    public boolean isRos2Mode() {
        return ros2Mode;
    }

    public void setRos2Mode(boolean ros2Mode) {
        this.ros2Mode = ros2Mode;
    }

    public List<String> getAdditionalPackages() {
        return additionalPackages;
    }

    public Optional<AdapterBluePrint> generateBluePrint(EMAComponentInstanceSymbol componentSymbol) {

        AdapterBluePrint bluePrint = null;

        if (TagHelper.rosConnectionsValid(componentSymbol)) {

            String name = NameHelper.getAdapterName(componentSymbol);
            bluePrint = new AdapterBluePrint(name, NameHelper.getComponentNameTargetLanguage(componentSymbol.getFullName()));
            List<EMAPortSymbol> rosPorts = componentSymbol.getPortInstanceList().stream()
                    .filter(EMAPortSymbol::isRosPort)
                    .collect(Collectors.toList());

            List<RosSubscriber> rosSubscribers = rosPorts.stream()
                    .filter(EMAPortSymbol::isIncoming)
                    .map(RosSubscriber::new)
                    .collect(Collectors.toList());

            List<RosPublisher> rosPublishers = rosPorts.stream()
                    .filter(EMAPortSymbol::isOutgoing)
                    .map(RosPublisher::new)
                    .collect(Collectors.toList());

            List<RosInterface> rosInterfaces = new ArrayList<>();
            rosInterfaces.addAll(rosSubscribers);
            rosInterfaces.addAll(rosPublishers);
            interfaces = rosInterfaces;

            SubscriberConverter subscriberConverter;
            PublisherConverter publisherConverter;
            if (ros2Mode) {
                subscriberConverter = new Ros2SubscriberConverter();
                publisherConverter = new Ros2PublisherConverter();
            } else {
                subscriberConverter = new RosSubscriberConverter();
                publisherConverter = new RosPublisherConverter();
            }

            generateFields(componentSymbol, rosSubscribers, rosPublishers, bluePrint, subscriberConverter, publisherConverter);
            generateCallbacks(rosSubscribers, bluePrint, subscriberConverter);
            generateInit(name, rosSubscribers, rosPublishers, bluePrint, subscriberConverter, publisherConverter);
            generatePublishMethods(rosPublishers, bluePrint, publisherConverter);
            generateTick(rosPublishers, bluePrint);
            generateIncludes(componentSymbol, rosInterfaces, bluePrint);
        }

        return Optional.ofNullable(bluePrint);

    }

    private void generateIncludes(EMAComponentInstanceSymbol componentSymbol, List<RosInterface> rosInterfaces, AdapterBluePrint bluePrint) {

        List<String> topicTypes = rosInterfaces.stream()
                .map(RosInterface::getRosConnectionSymbol)
                .map(RosConnectionSymbol::getTopicType)
                .map(Optional::get)
                .peek(topicType -> additionalPackages.add(NameHelper.getPackageOfMsgType(topicType)))
                .collect(Collectors.toList());

        if (!isRos2Mode()) {
            rosIncludes(componentSymbol, bluePrint, topicTypes);
        }else{
            ros2Includes(componentSymbol, bluePrint, topicTypes);
        }

        msgConverts.stream()
                .map(DirectMsgConverter::getAdditionalIncludes)
                .flatMap(Collection::stream)
                .distinct()
                .forEach(bluePrint::addAdditionalIncludeString);
    }

    private void rosIncludes(EMAComponentInstanceSymbol componentSymbol, AdapterBluePrint bluePrint, List<String> topicTypes) {
        bluePrint.addAdditionalIncludeString("<ros/ros.h>");
        String compName = NameHelper.getComponentNameTargetLanguage(componentSymbol.getFullName());
        bluePrint.addAdditionalIncludeString("\"" + compName + ".h\"");
        bluePrint.addAdditionalIncludeString("\"IAdapter_" + compName + ".h\"");
        //Add each msg include exactly once

        topicTypes.stream()
                .map(type -> "<" + type + ".h>")
                .forEach(bluePrint::addAdditionalIncludeString);
    }

    private void ros2Includes(EMAComponentInstanceSymbol componentSymbol, AdapterBluePrint bluePrint, List<String> topicTypes) {
        bluePrint.addAdditionalIncludeString("<rclcpp/rclcpp.hpp>");
        String compName = NameHelper.getComponentNameTargetLanguage(componentSymbol.getFullName());
        bluePrint.addAdditionalIncludeString("\"" + compName + ".h\"");
        bluePrint.addAdditionalIncludeString("\"IAdapter_" + compName + ".h\"");
        //Add each msg include exactly once

        topicTypes.stream()
                .map(type -> {
                    if (ros2Mode) {
                        return NameHelper.msgTypeToSnakecase(NameHelper.addMsgToMsgType(type));
                    } else {
                        return type;
                    }
                })
                .map(type -> "<" + type + ".hpp>")
                .forEach(bluePrint::addAdditionalIncludeString);
    }

    private void generatePublishMethods(List<RosPublisher> rosPublishers, AdapterBluePrint bluePrint, PublisherConverter publisherConverter) {
        rosPublishers.stream()
                .sorted(Comparator.comparing(a -> a.getPort().getName()))
                .forEachOrdered(p -> publisherConverter.convertToPublishMethod(p, bluePrint));
    }


    public void generateInit(String classname, List<RosSubscriber> subscribers, List<RosPublisher> publishers, AdapterBluePrint bluePrint, SubscriberConverter subscriberConverter, PublisherConverter publisherConverter) {
        Method initMethod = bluePrint.getInit();
        InitHelper.getInitInstructions(classname,isRos2Mode()).forEach(initMethod::addInstruction);

        //subs
        subscribers.stream()
                .sorted(Comparator.comparing(RosInterface::getNameInTargetLanguage))
                .forEach(s -> subscriberConverter.convertToInit(s, classname, bluePrint));

        publishers.stream()
                .sorted(Comparator.comparing(RosInterface::getNameInTargetLanguage))
                .forEach(p -> publisherConverter.convertToInit(p, bluePrint));

        if(!isRos2Mode()) {
            initMethod.addInstruction(new TargetCodeInstruction("ros::spin();"));
        }else{
            initMethod.addInstruction(new TargetCodeInstruction("rclcpp::spin(node_handle);"));
        }
    }

    private void generateFields(EMAComponentInstanceSymbol symbol, List<RosSubscriber> rosSubscribers, List<RosPublisher> rosPublishers, AdapterBluePrint bluePrint, SubscriberConverter subscriberConverter, PublisherConverter publisherConverter) {
        bluePrint.addDefineGenerics(symbol);
        rosSubscribers.forEach(rosSubscriber -> subscriberConverter.convertToField(rosSubscriber, bluePrint));
        rosPublishers.forEach(rosPublisher -> publisherConverter.convertToField(rosPublisher, bluePrint));
    }


    private void generateTick(List<RosPublisher> publishers, AdapterBluePrint bluePrint) {
        Method tickMethod = bluePrint.getTick();

        publishers.stream()
                .map(RosPublisher::getMethodName)
                .sorted()
                .map(CallMethodInstruction::new)
                .forEachOrdered(tickMethod::addInstruction);
    }

    private void generateCallbacks(List<RosSubscriber> rosSubscribers, AdapterBluePrint bluePrint, SubscriberConverter subscriberConverter) {
        rosSubscribers.forEach(s -> subscriberConverter.convertToCallback(s, bluePrint));
    }

    public Map<RosMsg, MCTypeReference<? extends MCTypeSymbol>> getUsedRosMsgs() {
        return usedRosMsgs;
    }

    public List<RosInterface> getRosInterfaces() {
        return interfaces;
    }
}
