package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import de.monticore.lang.monticar.generator.cpp.dynamic_interface.ProgramInterfaceResolver.SocketInfo;
import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;
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
            if (portInfo.direction == PortDirection.INPUT){
                setPortCases.add(generateSetPortCase(i, portInfo));
            } else {
                getPortCases.add(generateGetPortCase(i, portInfo));
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

            SocketInfo sockInf = gen.interfaceResolver.getSocketInfo(portInfo);
            int indent = BASE_INDENT+1;
            b.a(indent, "auto id = get_socket_id(br, %d);", sockInf.array_length);
            b.a(indent, "if (id < 0) return;");
            b.a(indent, "auto &target = program_instance.%s[id];", sockInf.input_name);
            generateSetter(((SimplePacketType) portInfo.data_type).getPayloadType(), BASE_INDENT+1, 1, "target");

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
            SocketInfo sockInf = gen.interfaceResolver.getSocketInfo(portInfo);
            int indent = BASE_INDENT+1;

            if (sockInf.is_bc) {
                b.a(indent, "static bool sent = false;");
                b.a(indent, "if (sent) {", sockInf.array_length);
                b.a(indent, "    sent = false;");
                b.a(indent, "    return;");
                b.a(indent, "}", sockInf.array_length);
                b.a(indent, "writer.write_str(N_TO_N_BROADCAST_ADDR); // Write address");
                b.a(indent, "auto &target = program_instance.%s;", sockInf.output_name);
                b.a(indent, "sent = true;");
            } else {
                b.a(indent, "static int id = 0;");
                b.a(indent, "if (id >= %d) {", sockInf.array_length);
                b.a(indent, "    id = 0;");
                b.a(indent, "    return;");
                b.a(indent, "}", sockInf.array_length);
                b.a(indent, "writer.write_str(N_TO_N_PREFIX + std::to_string(id+1)); // Write address");
                b.a(indent, "auto &target = program_instance.%s[id];", sockInf.output_name);
                b.a(indent, "++id;");
            }
            generateGetter(((SimplePacketType) portInfo.data_type).getPayloadType(), BASE_INDENT+1, 1, "target");

        }


        b.a(BASE_INDENT, "} break;");
        return b.getContent();
    }

    protected abstract void generateSetter(DataType type, int indent, int depth, String varReference);
    protected abstract void generateGetter(DataType type, int indent, int depth, String varReference);
    abstract String getType();
}
