package de.monticore.lang.monticar.generator.roscpp;

import com.google.common.base.Objects;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import java.util.HashSet;
import java.util.Set;

//TODO: rename: not used as a Tag
public class ResolvedRosTag {
    private EMAComponentInstanceSymbol component;
    private Set<ResolvedRosInterface> subscriberInterfaces = new HashSet<>();
    private Set<ResolvedRosInterface> publisherInterfaces = new HashSet<>();

    public ResolvedRosTag(EMAComponentInstanceSymbol component) {
        this.component = component;
    }

    public EMAComponentInstanceSymbol getComponent() {
        return component;
    }

    public void setComponent(EMAComponentInstanceSymbol component) {
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
