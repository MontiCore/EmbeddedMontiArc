/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.template;

import de.monticore.lang.monticar.generator.roscpp.util.RosInterface;
import de.monticore.lang.monticar.generator.roscpp.util.RosPublisher;
import de.monticore.lang.monticar.generator.roscpp.util.RosSubscriber;

import java.util.*;
import java.util.stream.Collectors;

public class RosAdapterModel {
    private String compName;
    private boolean ros2mode;
    private List<RosSubscriber> subscribers = new ArrayList<>();
    private List<RosPublisher> publishers = new ArrayList<>();
    private List<String> includes = new ArrayList<>();
    private List<String> generics = new ArrayList<>();

    public RosAdapterModel(boolean ros2mode, String compName) {
        this.ros2mode = ros2mode;
        this.compName = compName;
    }

    public boolean isRos2Mode() {
        return ros2mode;
    }

    public String getCompName() {
        return compName;
    }

    public List<String> getIncludes() {
        return includes.stream()
                .sorted()
                .distinct()
                .collect(Collectors.toList());
    }

    public List<RosSubscriber> getSubscribers() {
        return subscribers.stream()
                .sorted(Comparator.comparing(RosInterface::getNameInTargetLanguage))
                .collect(Collectors.toList());
    }

    public List<RosPublisher> getPublishers() {
        return publishers.stream()
                .sorted(Comparator.comparing(RosInterface::getNameInTargetLanguage))
                .collect(Collectors.toList());
    }

    public List<String> getGenerics() {
        return generics;
    }

    public void addInclude(String include){
        this.includes.add(include);
    }

    public boolean addSubscriber(RosSubscriber rosSubscriber) {
        return subscribers.add(rosSubscriber);
    }

    public boolean addSubscribers(Collection<? extends RosSubscriber> collection) {
        return subscribers.addAll(collection);
    }

    public boolean addPublisher(RosPublisher rosPublisher) {
        return publishers.add(rosPublisher);
    }

    public boolean addPublishers(Collection<? extends RosPublisher> collection) {
        return publishers.addAll(collection);
    }

    public boolean addGenerics(Collection<? extends String> collection){
        return generics.addAll(collection);
    }

    public String getMw(){
        return isRos2Mode() ? "ROS2" : "ROS";
    }

}
