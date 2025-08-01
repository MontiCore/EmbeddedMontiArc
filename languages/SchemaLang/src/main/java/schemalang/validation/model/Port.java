package schemalang.validation.model;

public class Port {

    private String portName;
    private PortType portType;
    private PortDirection portDirection;

    public String getPortName() {
        return portName;
    }

    public void setPortName(String portName) {
        this.portName = portName;
    }

    public PortType getPortType() {
        return portType;
    }

    public void setPortType(PortType portType) {
        this.portType = portType;
    }

    public PortDirection getPortDirection() {
        return portDirection;
    }

    public void setPortDirection(PortDirection portDirection) {
        this.portDirection = portDirection;
    }
}