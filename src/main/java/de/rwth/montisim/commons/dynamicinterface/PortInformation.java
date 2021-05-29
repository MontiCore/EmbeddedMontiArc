/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.dynamicinterface;

import java.util.Vector;

public class PortInformation {
    public static enum PortDirection {
        INPUT,
        OUTPUT,
        IO // Both, example: for a socket that can send and receive the same message type.
    }
    public static enum PortType {
        DATA,
        SOCKET
    }

    public String name;
    public PortType port_type = PortType.DATA;
    public PortDirection direction;
    public DataType data_type;
    public boolean allows_multiple_inputs = true;
    public boolean optional = false;
    public Vector<String> tags = new Vector<>(); // Tags for routing. Example: 'network' tag -> gets collected by the Gateway component

    protected PortInformation() {}
    
    protected PortInformation(String name, PortType port_type, DataType data_type, PortDirection dir, boolean multipleInputsAllowed, boolean optional){
        this.name = name;
        this.port_type = port_type;
        this.data_type = data_type;
        this.direction = dir;
        this.allows_multiple_inputs = multipleInputsAllowed;
        this.optional = optional;
    }

    public static PortInformation newInputDataPort(String name, DataType dataType, boolean multipleInputsAllowed, boolean optional) {
        return new PortInformation(name, PortType.DATA, dataType, PortDirection.INPUT, multipleInputsAllowed, optional);
    }
    public static PortInformation newOptionalInputDataPort(String name, DataType dataType, boolean multipleInputsAllowed) {
        return new PortInformation(name, PortType.DATA, dataType, PortDirection.INPUT, multipleInputsAllowed, true);
    }
    public static PortInformation newRequiredInputDataPort(String name, DataType dataType, boolean multipleInputsAllowed) {
        return new PortInformation(name, PortType.DATA, dataType, PortDirection.INPUT, multipleInputsAllowed, false);
    }
    public static PortInformation newOutputDataPort(String name, DataType dataType, boolean optional) {
        return new PortInformation(name, PortType.DATA, dataType, PortDirection.OUTPUT, true, optional);
    }
    public static PortInformation newOptionalOutputDataPort(String name, DataType dataType) {
        return new PortInformation(name, PortType.DATA, dataType, PortDirection.OUTPUT, true, true);
    }
    public static PortInformation newRequiredOutputDataPort(String name, DataType dataType) {
        return new PortInformation(name, PortType.DATA, dataType, PortDirection.OUTPUT, true, false);
    }
    public static PortInformation newSocketPort(String name, DataType dataType, boolean in, boolean out) {
        if (!in && !out) throw new IllegalArgumentException("newSocketPort() with neither 'in' or 'out'");
        PortDirection pdir = in ? out ? PortDirection.IO : PortDirection.INPUT : PortDirection.OUTPUT;
        return new PortInformation(name, PortType.SOCKET, dataType, pdir, true, true);
    }

    public PortInformation addTag(String tag) {
        this.tags.add(tag);
        return this;
    }

    public boolean isInput() {
        return direction == PortDirection.INPUT || direction == PortDirection.IO;
    }
    public boolean isOutput() {
        return direction == PortDirection.OUTPUT || direction == PortDirection.IO;
    }
}
