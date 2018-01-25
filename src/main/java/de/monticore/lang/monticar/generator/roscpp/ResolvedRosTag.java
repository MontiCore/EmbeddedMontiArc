package de.monticore.lang.monticar.generator.roscpp;

import com.google.common.base.Objects;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;

import java.util.HashSet;
import java.util.Set;

public class ResolvedRosTag {
    private ExpandedComponentInstanceSymbol component;
    private Set<ResolvedRosInterface> subscriberInterfaces = new HashSet<>();
    private Set<ResolvedRosInterface> publisherInterfaces = new HashSet<>();

    public ResolvedRosTag(ExpandedComponentInstanceSymbol component) {
        this.component = component;
    }

    public ExpandedComponentInstanceSymbol getComponent() {
        return component;
    }

    public void setComponent(ExpandedComponentInstanceSymbol component) {
        this.component = component;
    }

    public void addSubscriberInterface(ResolvedRosInterface subInterface) {
        subscriberInterfaces.add(subInterface);
    }

    public void addPublisherInterface(ResolvedRosInterface pubInterface) {
        publisherInterfaces.add(pubInterface);
    }

    public Set<ResolvedRosInterface> getSubscriberInterfaces() {
        return subscriberInterfaces;
    }

    public Set<ResolvedRosInterface> getPublisherInterfaces() {
        return publisherInterfaces;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        ResolvedRosTag that = (ResolvedRosTag) o;
        return Objects.equal(component, that.component) &&
                Objects.equal(subscriberInterfaces, that.subscriberInterfaces) &&
                Objects.equal(publisherInterfaces, that.publisherInterfaces);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(component, subscriberInterfaces, publisherInterfaces);
    }
}
