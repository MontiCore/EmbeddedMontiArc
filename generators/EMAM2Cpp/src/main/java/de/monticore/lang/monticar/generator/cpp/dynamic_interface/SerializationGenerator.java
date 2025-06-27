package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortType;
import de.rwth.montisim.commons.utils.json.SerializationException;

public abstract class SerializationGenerator {
    static final int BASE_INDENT = 2;

    protected final AdapterGenerator gen;

    protected final FileBuilder b = new FileBuilder();
    
    SerializationGenerator(AdapterGenerator gen) {
        this.gen = gen;
    }

    public void generate(HashMap<String, Object> serializationTemplateData) throws SerializationException {
        List<String> getPortCases = new ArrayList<>();
        List<String> setPortCases = new ArrayList<>();

        int i = 0;
        for (PortInformation portInfo : gen.interfaceResolver.programInterface.ports){
            if (portInfo.isOutput()) {
                getPortCases.add(generateGetPortCase(i, portInfo));
            }
            if (portInfo.isInput()) {
                setPortCases.add(generateSetPortCase(i, portInfo));
            }
            ++i;
        }

        serializationTemplateData.put("getPort"+getType()+"Cases", getPortCases);
        serializationTemplateData.put("setPort"+getType()+"Cases", setPortCases);

    }
    
    
    private String generateSetPortCase(int id, PortInformation portInfo){
        b.init();
        b.a(BASE_INDENT, "case %d: { // %s", id, portInfo.name);

        if (portInfo.port_type == PortType.DATA) {
            generateSetter(portInfo.data_type, BASE_INDENT+1, 1, "program_instance."+portInfo.name);
        } else {
            generateSocketSetter(portInfo);
        }

        b.a(BASE_INDENT, "} break;");
        
        return b.getContent();
    }

    private String generateGetPortCase(int id, PortInformation portInfo){
        b.init();
        b.a(BASE_INDENT, "case %d: { // %s", id, portInfo.name);

        if (portInfo.port_type == PortType.DATA) {
            generateGetter(portInfo.data_type, BASE_INDENT+1, 1, "program_instance."+portInfo.name);
        } else {
            generateSocketGetter(portInfo);
        }

        b.a(BASE_INDENT, "} break;");
        return b.getContent();
    }

    protected abstract void generateSetter(DataType type, int indent, int depth, String varReference);
    protected abstract void generateGetter(DataType type, int indent, int depth, String varReference);
    abstract void generateSocketGetter(PortInformation portInfo);
    abstract void generateSocketSetter(PortInformation portInfo);
    abstract String getType();
    abstract boolean isJson(); // Not ideal
}
