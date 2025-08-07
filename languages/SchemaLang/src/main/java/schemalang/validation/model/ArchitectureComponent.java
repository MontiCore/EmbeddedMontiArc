package schemalang.validation.model;

import com.google.common.collect.Lists;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class ArchitectureComponent {

    private String fullName;
    private List<Port> ports = Lists.newArrayList();

    public String getFullName() {
        return fullName;
    }

    public void setFullName(String fullName) {
        this.fullName = fullName;
    }

    public List<Port> getPorts() {
        return ports;
    }

    public List<Port> getIncomingPorts() {
        return ports.stream().filter(p -> PortDirection.INPUT.equals(p.getPortDirection())).
                collect(Collectors.toList());
    }

    public List<Port> getOutgoingPorts() {
        return ports.stream().filter(p -> PortDirection.OUTPUT.equals(p.getPortDirection())).
                collect(Collectors.toList());
    }

    public Optional<Port> getPort(String name) {
        return ports.stream().filter(p -> p.getPortName().equals(name)).findFirst();
    }

    public void addPort(Port port) {
        ports.add(port);
    }

    public Optional<Port> getIncomingPort(String name) {
        return ports.stream().filter(p -> PortDirection.INPUT.equals(p.getPortDirection())
                && p.getPortName().equals(name)).findFirst();
    }

    public Optional<Port> getOutgoingPort(String name) {
        return ports.stream().filter(p -> PortDirection.OUTPUT.equals(p.getPortDirection())
                && p.getPortName().equals(name)).findFirst();
    }

    public void setPorts(List<Port> ports) {
        this.ports = ports;
    }
}