package de.monticore.lang.monticar.generator.roscpp;

import com.google.common.base.Objects;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.Variable;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class RosTopic {
    private String name;
    private String rosType;
    private String importString;
    private Optional<Variable> subscriber = Optional.empty();
    private Optional<Variable> publisher = Optional.empty();
    private Optional<Method> callback = Optional.empty();
    private Optional<Method> publishMethod = Optional.empty();
    private Set<PortSymbol> ports = new HashSet<>();


    public RosTopic(String name, String rosType, String importString) {
        this.name = name;
        this.rosType = rosType;
        this.importString = importString;
    }

    public void addPort(PortSymbol portSymbol) {
        ports.add(portSymbol);
    }

    public Set<PortSymbol> getPorts() {
        return ports;
    }

    public Set<PortSymbol> getIncommingPorts() {
        return ports.stream().filter(PortSymbol::isIncoming).collect(Collectors.toSet());
    }

    public Set<PortSymbol> getOutgoingPorts() {
        return ports.stream().filter(PortSymbol::isOutgoing).collect(Collectors.toSet());
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getRosType() {
        return rosType;
    }

    public void setRosType(String rosType) {
        this.rosType = rosType;
    }

    public String getImportString() {
        return importString;
    }

    public void setImportString(String importString) {
        this.importString = importString;
    }

    public Optional<Variable> getSubscriber() {
        return subscriber;
    }

    public Optional<Variable> getPublisher() {
        return publisher;
    }

    public Optional<Method> getCallback() {
        return callback;
    }

    public void setSubscriber(Variable subscriber) {
        this.subscriber = Optional.of(subscriber);
    }

    public void setPublisher(Variable publisher) {
        this.publisher = Optional.of(publisher);
    }

    public void setCallback(Method callback) {
        this.callback = Optional.of(callback);
    }

    public String getTargetLanguageName() {
        return this.getName().replace('/', '_');
    }

    public void setPublishMethod(Method method) {
        publishMethod = Optional.ofNullable(method);
    }

    public Optional<Method> getPublishMethod() {
        return publishMethod;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        RosTopic rosTopic = (RosTopic) o;
        return Objects.equal(name, rosTopic.name) &&
                Objects.equal(rosType, rosTopic.rosType) &&
                Objects.equal(importString, rosTopic.importString);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(name, rosType, importString);
    }

    //TODO: unclean
    public String getFullRosType() {
        return importString.replace("/", "::");
    }
}
