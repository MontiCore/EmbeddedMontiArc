package de.monticore.lang.monticar.generator.roscpp;

import com.google.common.base.Objects;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.Variable;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class ResolvedRosInterface {
    private String type;
    private String topic;
    private String include;
    private Map<PortSymbol, String> portToMsgField = new HashMap<>();
    private Optional<Variable> publisherField = Optional.empty();
    private Optional<Variable> subscriberField = Optional.empty();
    private Optional<Method> publishMethod = Optional.empty();

    public void setPublishMethod(Method publishMethod) {
        this.publishMethod = Optional.of(publishMethod);
    }

    public Optional<Method> getPublishMethod() {
        return publishMethod;
    }

    public void setPublisherField(Variable publisherField) {
        this.publisherField = Optional.of(publisherField);
    }

    public void setSubscriberField(Variable subscriberField) {
        this.subscriberField = Optional.of(subscriberField);
    }

    public Optional<Variable> getPublisherField() {
        return publisherField;
    }

    public Optional<Variable> getSubscriberField() {
        return subscriberField;
    }

    public ResolvedRosInterface(String type, String topic, String include) {
        this.type = type;
        this.topic = topic;
        this.include = include;
    }

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }

    public String getTopic() {
        return topic;
    }

    public String getTargetLanguageName() {
        return this.getTopic().replace('/', '_');
    }

    public String getFullRosType() {
        return include.replace("/", "::");
    }

    public void setTopic(String topic) {
        this.topic = topic;
    }

    public String getInclude() {
        return include;
    }

    public void setInclude(String include) {
        this.include = include;
    }

    public void addPort(PortSymbol portSymbol, String msgField) {
        portToMsgField.put(portSymbol, msgField);
    }

    public Set<PortSymbol> getPorts() {
        return portToMsgField.keySet();
    }

    public String getMsgFieldForPort(PortSymbol portSymbol) {
        return portToMsgField.get(portSymbol);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        ResolvedRosInterface that = (ResolvedRosInterface) o;
        return Objects.equal(type, that.type) &&
                Objects.equal(topic, that.topic) &&
                Objects.equal(include, that.include) &&
                Objects.equal(portToMsgField, that.portToMsgField);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(type, topic, include, portToMsgField);
    }
}
