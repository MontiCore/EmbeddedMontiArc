package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;
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

        int i = 0;
        for (PortInformation portInfo : gen.programInterface.ports){
            if (portInfo.direction == PortDirection.INPUT){
                setInputCases.add(generateSetInputCase(i, portInfo));
            } else {
                sendOutputCalls.add(String.format("send_output(%d);", i));
                sendOutputCases.add(generateSendOutputCase(i, portInfo));
            }
            ++i;
        }

        HashMap<String, Object> templateData = new HashMap<>();
        templateData.put("mainModelName", gen.componentName);
        templateData.put("interfaceDescription", gen.progInterfaceString);
        templateData.put("setInputCases", setInputCases);
        templateData.put("sendOutputCalls", sendOutputCalls);
        templateData.put("sendOutputCases", sendOutputCases);
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

        b.a(BASE_INDENT, "case %d: { // %s", id, portInfo.name);
        generateSetter(portInfo.type, BASE_INDENT+1, 1, "program_instance."+portInfo.name);
        b.a(BASE_INDENT, "} break;");

        return b.getContent();
    }

    public String generateSendOutputCase(int id, PortInformation portInfo){
        b.init();

        b.a(BASE_INDENT, "case %d: { // %s", id, portInfo.name);
        generateGetter(portInfo.type, BASE_INDENT+1, 1, "program_instance."+portInfo.name);
        b.a(BASE_INDENT, "} break;");

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
            String i = String.format("i%d", depth);
            String ref = String.format("res%d", depth);

            
            b.a(indent, "for (int %s=0; %s < %d; ++%s) {",   i, i, size, i);
            b.a(indent, "    auto &%s = %s(%s);",            ref, varReference, i);
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


}
