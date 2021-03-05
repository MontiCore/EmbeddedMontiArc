package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.dynamic_interface.DynamicInterfaceGenerator.SocketInfo;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortType;
import de.rwth.montisim.commons.utils.json.SerializationException;

public class TcpCommunication {
    static final int BASE_INDENT = 2;

    final DynamicInterfaceGenerator gen;

    FileBuilder b = new FileBuilder();

    
    public TcpCommunication(DynamicInterfaceGenerator gen) {
        this.gen = gen;
    }


    public List<FileContent> generate(boolean hasDDC) throws SerializationException {
        List<FileContent> files = new ArrayList<>();
        List<String> setInputCases = new ArrayList<>();
        List<String> sendOutputCalls = new ArrayList<>();
        List<String> sendOutputCases = new ArrayList<>();
        List<String> sendSocketOutputsCases = new ArrayList<>();
        List<String> initDataCalls = new ArrayList<>();

        int i = 0;
        for (PortInformation portInfo : gen.programInterface.ports){
            initDataCalls.add(generateInitData(portInfo));
            if (portInfo.isInput()){
                setInputCases.add(generateSetInputCase(i, portInfo));
            } 
            if (portInfo.isOutput()) {
                if (portInfo.port_type == PortType.DATA) {
                    sendOutputCalls.add(String.format("send_output(%d);", i));
                    sendOutputCases.add(generateSendOutputCase(i, portInfo));
                } else {
                    sendOutputCalls.add(String.format("send_socket_outputs(%d);", i));
                    sendSocketOutputsCases.add(generateSendSocketOutputsCase(i, portInfo));
                }
            }
            ++i;
        }

        HashMap<String, Object> templateData = new HashMap<>();
        templateData.put("mainModelName", gen.componentName);
        templateData.put("interfaceDescription", gen.progInterfaceString);
        templateData.put("setInputCases", setInputCases);
        templateData.put("sendOutputCalls", sendOutputCalls);
        templateData.put("sendOutputCases", sendOutputCases);
        templateData.put("sendSocketOutputsCases", sendSocketOutputsCases);
        templateData.put("initDataCalls", initDataCalls);
        templateData.put("useDDC", hasDDC);

        files.add(new FileContent(AllTemplates.generate(AllTemplates.TCP_ADAPTER_H, templateData), "server_adapter.h"));
        files.add(new FileContent(AllTemplates.generate(AllTemplates.TCP_ADAPTER_CPP, templateData), "server_adapter.cpp"));

        gen.addCppFileDependency("json.h");
        gen.addCppFileDependency("json.cpp");
        gen.addCppFileDependency("printf.h");
        gen.addCppFileDependency("printf.cpp");
        gen.addCppFileDependency("utils.h");
        gen.addCppFileDependency("network.h");
        gen.addCppFileDependency("network.cpp");
        gen.addCppFileDependency("tcp_protocol.h");

        return files;
    }

	

    public void getSources(HashSet<String> sources) {
        sources.add("json.cpp");
        sources.add("printf.cpp");
        sources.add("network.cpp");
        sources.add("server_adapter.cpp");
    }
    public void getLibs(HashSet<String> libs) {

    }



    public String generateSetInputCase(int id, PortInformation portInfo){
        b.init();

        if (portInfo.port_type == PortType.DATA) {

            b.a(BASE_INDENT, "case %d: { // %s", id, portInfo.name);
            generateSetter(portInfo.data_type, BASE_INDENT+1, 1, "program_instance."+portInfo.name);
            b.a(BASE_INDENT, "} break;");

        } else {
            SocketInfo sockInf = gen.getSocketInfo(portInfo);
            b.a(BASE_INDENT, "case %d: { // %s", id, portInfo.name);
            int indent = BASE_INDENT+1;
            b.a(indent, "auto id = get_socket_id(input_packet, %d);", sockInf.array_length);
            b.a(indent, "if (id < 0) return;");
            b.a(indent, "auto &target = program_instance.%s[id];", sockInf.input_name);
            generateSetter(((SimplePacketType) portInfo.data_type).getPayloadType(), BASE_INDENT+1, 1, "target");
            b.a(BASE_INDENT, "} break;");

        }

        return b.getContent();
    }

    public String generateSendOutputCase(int id, PortInformation portInfo){
        b.init();

        b.a(BASE_INDENT, "case %d: { // %s", id, portInfo.name);
        generateGetter(portInfo.data_type, BASE_INDENT+1, 1, "program_instance."+portInfo.name);
        b.a(BASE_INDENT, "} break;");

        return b.getContent();
    }

    

    private String generateSendSocketOutputsCase(int id, PortInformation portInfo) {
        b.init();
        b.a(BASE_INDENT, "case %d: { // %s", id, portInfo.name);
        SocketInfo sockInf = gen.getSocketInfo(portInfo);
        if (sockInf.is_bc) {
            b.a(BASE_INDENT+1, "PacketWriter packet(buffer, BUFFER_SIZE, PACKET_OUTPUT);");
            b.a(BASE_INDENT+1, "packet.write_u16(port_id);");
            b.a(BASE_INDENT+1, "// Payload");
            b.a(BASE_INDENT+1, "packet.write_str(N_TO_N_BROADCAST_ADDR); // Write address");
            generateGetter(((SimplePacketType) portInfo.data_type).getPayloadType(), BASE_INDENT+1, 1, "program_instance."+sockInf.output_name);
            b.a(BASE_INDENT+1, "packet.send(socket);");
        } else {
            b.a(BASE_INDENT+1, "for (int i = 0; i < %d; ++i) {", sockInf.array_length);
            b.a(BASE_INDENT+1, "    PacketWriter packet(buffer, BUFFER_SIZE, PACKET_OUTPUT);");
            b.a(BASE_INDENT+1, "    packet.write_u16(port_id);");
            b.a(BASE_INDENT+1, "    // Payload");
            b.a(BASE_INDENT+1, "    packet.write_str(N_TO_N_PREFIX + to_string(i+1)); // Write address");
            b.a(BASE_INDENT+1, "    auto &target = program_instance.%s[i];", sockInf.output_name);
            generateGetter(((SimplePacketType) portInfo.data_type).getPayloadType(), BASE_INDENT+2, 1, "target");
            b.a(BASE_INDENT+1, "    packet.send(socket);");
            b.a(BASE_INDENT+1, "}");
        }
        b.a(BASE_INDENT, "} break;");
        return b.getContent();
    }

    
    private String generateInitData(PortInformation portInfo) {
        b.init();
        if (portInfo.port_type == PortType.DATA) {
            generateInitData(portInfo.data_type, 1, 1, "program_instance."+portInfo.name);
        } else {
            DataType dt = ((SimplePacketType) portInfo.data_type).getPayloadType();
            SocketInfo sockInf = gen.getSocketInfo(portInfo);

            if (portInfo.isInput()) {
                for (int i =0; i < sockInf.array_length; ++i) {
                    String varName = sockInf.input_name + Integer.toString(i);
                    b.a(1, "auto &%s = program_instance.%s[%d];", varName, sockInf.input_name, i);
                    generateInitData(dt, 1, 1, varName);
                }
            }
            if (portInfo.isOutput()) {
                if (sockInf.is_bc) {
                    generateInitData(dt, 1, 1, "program_instance."+sockInf.output_name);
                } else {
                    for (int i =0; i < sockInf.array_length; ++i) {
                        String varName = sockInf.output_name + Integer.toString(i);
                        b.a(1, "auto &%s = program_instance.%s[%d];", varName, sockInf.output_name, i);
                        generateInitData(dt, 1, 1, varName);
                    }
                }
                
            }
            
        }
        return b.getContent();
    }



    private void generateSetter(DataType type, int indent, int depth, String varReference){

        if (type instanceof BasicType){
            BasicType t = (BasicType) type;
            switch (t.getType()){
                case Q:
                b.a(indent, "%s = input_packet.read_f64();", varReference);
                    break;
                case Z:
                case N:
                case N1: // TOTO check range for N1 & N ?
                b.a(indent, "%s = (int32_t) input_packet.read_u32();", varReference);
                    break;
                case C:
                b.a(indent, "%s = input_packet.read_f64();", varReference);
                b.a(indent, "%s = input_packet.read_f64();", varReference); // TODO correct complex number native case
                throw new IllegalStateException("Unimplemented");
                case BOOLEAN:
                b.a(indent, "%s = input_packet.read_u8() != 0;", varReference);
                    break;
                case EMPTY:
                    break;
                case VEC2:
                b.a(indent, "%s(0) = input_packet.read_f64();", varReference);
                b.a(indent, "%s(1) = input_packet.read_f64();", varReference);
                    break;
                case VEC3:
                b.a(indent, "%s(0) = input_packet.read_f64();", varReference);
                b.a(indent, "%s(1) = input_packet.read_f64();", varReference);
                b.a(indent, "%s(2) = input_packet.read_f64();", varReference);
                    break;
                default:
                throw new IllegalArgumentException("Missing case");
            }
        } else if (type instanceof VectorType) {
            VectorType vt = (VectorType)type;
            int newIndent = indent + 1;
            int size = vt.getSize();
            String iVar = String.format("i%d", depth);
            String sizeVar = String.format("size%d", depth);
            String ref = String.format("res%d", depth);

            // TODO check in code if the received size is within the declared max size?
            b.a(indent, "auto %s = input_packet.read_u16();", sizeVar);
            b.a(indent, "for (int %s=0; %s < %s; ++%s) {",    iVar, iVar, sizeVar, iVar);
            b.a(indent, "    auto &%s = %s(%s);",             ref, varReference, iVar);
            generateSetter(vt.getBaseType(), newIndent, depth+1, ref);
            b.a(indent, "}"                                  );
            
        } else if (type instanceof MatrixType) {
            MatrixType vt = (MatrixType)type;
            int newIndent = indent + 2;
            int rows = vt.getRowCount();
            int cols = vt.getColumnCount();
            String i = String.format("i%d", depth);
            String j = String.format("j%d", depth);
            String ref = String.format("res%d", depth);

            b.a(indent, "for (int %s=0; %s < %d; ++%s) {",       i, i, rows, i);
            b.a(indent, "    for (int %s=0; %s < %d; ++%s) {",   j, j, cols, j);
            b.a(indent, "        auto &%s = %s(%s, %s);",        ref, varReference, i, j);
            generateSetter(vt.getBaseType(), newIndent, depth+1, ref);
            b.a(indent, "    }"                                  );
            b.a(indent, "}"                                      );
            throw new IllegalArgumentException("TODO read sizes first?");

        } else if (type instanceof DynVectorType) {
            System.out.println("Unimplemented: DynVectorType set_port case.");
            //throw new IllegalArgumentException("Unimplemented");
        } else if (type instanceof StructType) {
            StructType st = (StructType)type;
            int count = st.getFieldCount();

            for (int i = 0; i < count; ++i){
                String fieldName = st.getFieldName(i);
                generateSetter(st.getFieldType(i), indent, depth+1, String.format("%s.%s", varReference, fieldName));
            }
        } else if (type instanceof EnumType) {
            EnumType et = (EnumType)type;
            
            int variantCount = et.getVariantCount();
            if (variantCount > 256) throw new IllegalArgumentException("Too much Enum variants for enum "+et.getName()+ " (TODO implement adaptive native type size)");

            b.a(indent, "%s = static_cast<%s>(input_packet.read_u8());", et.getName());
        } else {
            throw new IllegalArgumentException("Missing case");
        }

    }

    private void generateGetter(DataType type, int indent, int depth, String varReference) {

        if (type instanceof BasicType){
            BasicType t = (BasicType) type;
            switch (t.getType()){
                case Q:
                b.a(indent, "packet.write_f64(%s);", varReference);
                break;
                case Z:
                case N:
                case N1:
                b.a(indent, "packet.write_u32(%s);", varReference);
                case BOOLEAN:
                b.a(indent, "packet.write_u8(%s ? 1 : 0);", varReference);
                    break;
                case EMPTY:
                    break;
                case C:
                b.a(indent, "packet.write_f64(%s);", varReference);
                b.a(indent, "packet.write_f64(0);");
                throw new IllegalStateException("Unimplemented: Missing Proper native Complex Type");
                case VEC2:
                b.a(indent, "packet.write_f64(%s(0));", varReference);
                b.a(indent, "packet.write_f64(%s(1));", varReference);
                    break;
                case VEC3:
                b.a(indent, "packet.write_f64(%s(0));", varReference);
                b.a(indent, "packet.write_f64(%s(1));", varReference);
                b.a(indent, "packet.write_f64(%s(2));", varReference);
                    break;
                default:
                throw new IllegalArgumentException("Missing case");
            }
        } else if (type instanceof VectorType) {
            VectorType vt = (VectorType)type;
            int newIndent = indent + 1;
            int size = vt.getSize();
            String i = String.format("i%d", depth);
            String ref = String.format("res%d", depth);

            b.a(indent, "packet.write_u16(%d);", size);
            b.a(indent, "for (int %s=0; %s < %d; ++%s) {",   i, i, size, i);
            b.a(indent, "    auto &%s = %s(%s);",            ref, varReference, i);
            generateGetter(vt.getBaseType(), newIndent, depth+1, ref);
            b.a(indent, "}"                                  );

        } else if (type instanceof MatrixType) {
            MatrixType vt = (MatrixType)type;
            int newIndent = indent + 2;
            int rows = vt.getRowCount();
            int cols = vt.getColumnCount();
            String i = String.format("i%d", depth);
            String j = String.format("j%d", depth);
            String ref = String.format("res%d", depth);

            b.a(indent, "for (int %s=0; %s < %d; ++%s) {",       i, i, rows, i);
            b.a(indent, "    for (int %s=0; %s < %d; ++%s) {",   j, j, cols, j);
            b.a(indent, "        auto &%s = %s(%s, %s);",        ref, varReference, i, j);
            generateGetter(vt.getBaseType(), newIndent, depth+1, ref);
            b.a(indent, "    }"                                  );
            b.a(indent, "}"                                      );
            throw new IllegalArgumentException("TODO write sizes?");

        }else if (type instanceof DynVectorType) {
            System.out.println("Unimplemented: DynVectorType get_port case.");
            //throw new IllegalArgumentException("Unimplemented");
        } else if (type instanceof StructType) {
            StructType st = (StructType)type;
            int count = st.getFieldCount();

            
            for (int i = 0; i < count; ++i){
                String fieldName = st.getFieldName(i);
                generateGetter(st.getFieldType(i), indent, depth+1, String.format("%s.%s", varReference, fieldName));
            }
            
        } else if (type instanceof EnumType) {
            EnumType et = (EnumType)type;
            int variantCount = et.getVariantCount();
            if (variantCount > 256) throw new IllegalArgumentException("Too much Enum variants for enum "+et.getName()+ " (TODO implement adaptive native type size)");

            b.a(indent, "packet.write_u8(static_cast<uint8_t>(%s));", varReference);
        } else {
            throw new IllegalArgumentException("Missing case");
        }
    }





    private void generateInitData(DataType type, int indent, int depth, String varReference) {
        if (type instanceof BasicType){
            BasicType t = (BasicType) type;
            switch (t.getType()){
                case Q: 
                    b.a(indent, "%s = 0.0;", varReference);
                    return;
                case Z:
                case N:
                case N1:
                    b.a(indent, "%s = 0;", varReference);
                    return;
                case C:
                    b.a(indent, "%s = 0.0;", varReference);
                    // TODO correct complex number native case
                    throw new IllegalStateException("Unimplemented");
                case BOOLEAN:
                    b.a(indent, "%s = false;", varReference);
                    return;
                case EMPTY:
                    return;
                case VEC2:
                    b.a(indent, "%s(0) = 0.0;", varReference);
                    b.a(indent, "%s(1) = 0.0;", varReference);
                    return;
                case VEC3:
                    b.a(indent, "%s(0) = 0.0;", varReference);
                    b.a(indent, "%s(1) = 0.0;", varReference);
                    b.a(indent, "%s(2) = 0.0;", varReference);
                    return;
                default:
                throw new IllegalArgumentException("Missing case");
            }
        } else if (type instanceof VectorType) {
            VectorType vt = (VectorType)type;
            int newIndent = indent + 1;
            int size = vt.getSize();
            String iVar = String.format("i%d", depth);
            String ref = String.format("res%d", depth);

            b.a(indent, "for (int %s=0; %s < %d; ++%s) {",    iVar, iVar, size, iVar);
            b.a(indent, "    auto &%s = %s(%s);",             ref, varReference, iVar);
            generateInitData(vt.getBaseType(), newIndent, depth+1, ref);
            b.a(indent, "}"                                  );
            
        } else if (type instanceof MatrixType) {
            MatrixType vt = (MatrixType)type;
            int newIndent = indent + 2;
            int rows = vt.getRowCount();
            int cols = vt.getColumnCount();
            String i = String.format("i%d", depth);
            String j = String.format("j%d", depth);
            String ref = String.format("res%d", depth);

            
            b.a(indent, "for (int %s=0; %s < %d; ++%s) {",       i, i, rows, i);
            b.a(indent, "    for (int %s=0; %s < %d; ++%s) {",   j, j, cols, j);
            b.a(indent, "        auto &%s = %s(%s, %s);",        ref, varReference, i, j);
            generateInitData(vt.getBaseType(), newIndent, depth+1, ref);
            b.a(indent, "    }"                                  );
            b.a(indent, "}"                                      );

        } else if (type instanceof DynVectorType) {
            System.out.println("Unimplemented: DynVectorType generateInitData case.");
            //throw new IllegalArgumentException("Unimplemented");
        } else if (type instanceof StructType) {
            StructType st = (StructType)type;
            int count = st.getFieldCount();

            for (int i = 0; i < count; ++i){
                String fieldName = st.getFieldName(i);
                generateInitData(st.getFieldType(i), indent, depth+1, String.format("%s.%s", varReference, fieldName));
            }
        } else if (type instanceof EnumType) {
            EnumType et = (EnumType)type;
            
            int variantCount = et.getVariantCount();
            if (variantCount > 256) throw new IllegalArgumentException("Too much Enum variants for enum "+et.getName()+ " (TODO implement adaptive native type size)");

            b.a(indent, "%s = static_cast<%s>(0);", et.getName());
        } else {
            throw new IllegalArgumentException("Missing case");
        }
    }
}
