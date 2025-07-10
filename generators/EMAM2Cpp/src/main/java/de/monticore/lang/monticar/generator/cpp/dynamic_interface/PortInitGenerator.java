package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import de.monticore.lang.monticar.generator.cpp.dynamic_interface.ProgramInterfaceResolver.SocketInfo;
import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortType;
import de.rwth.montisim.commons.utils.json.SerializationException;

public class PortInitGenerator {
    static final int BASE_INDENT = 2;

    final AdapterGenerator gen;

    FileBuilder b = new FileBuilder();
    
    PortInitGenerator(AdapterGenerator gen) {
        this.gen = gen;
    }

    public void generate(HashMap<String, Object> serializationTemplateData) throws SerializationException {
        List<String> initDataCalls = new ArrayList<>();

        for (PortInformation portInfo : gen.interfaceResolver.programInterface.ports){
            initDataCalls.add(generateInitData(portInfo));
        }

        serializationTemplateData.put("initDataCalls", initDataCalls);
    }


    private String generateInitData(PortInformation portInfo) {
        int indent = 1;
        int depth = 1;
        b.init();
        if (portInfo.port_type == PortType.DATA) {
            generateInitData(portInfo.data_type, indent, depth, "program_instance."+portInfo.name);
        } else {
            DataType dt = ((SimplePacketType) portInfo.data_type).getPayloadType();
            SocketInfo sockInf = gen.interfaceResolver.getSocketInfo(portInfo);

            if (portInfo.isInput()) {
                int newIndent = indent + 1;
                String iVar = String.format("i%d", depth);
                String ref = String.format("res%d", depth);
                String varReference = "program_instance."+sockInf.input_name;

                b.a(indent, "for (int %s=0; %s < %d; ++%s) {",    iVar, iVar, sockInf.array_length, iVar);
                b.a(indent, "    auto &%s = %s[%s];",             ref, varReference, iVar);
                generateInitData(dt, newIndent, depth+1, ref);
                b.a(indent, "}");
            }
            if (portInfo.isOutput()) {
                if (sockInf.is_bc) {
                    generateInitData(dt, 1, 1, "program_instance."+sockInf.output_name);
                } else {
                    int newIndent = indent + 1;
                    String iVar = String.format("i%d", depth);
                    String ref = String.format("res%d", depth);
                    String varReference = "program_instance."+sockInf.output_name;

                    b.a(indent, "for (int %s=0; %s < %d; ++%s) {",    iVar, iVar, sockInf.array_length, iVar);
                    b.a(indent, "    auto &%s = %s[%s];",             ref, varReference, iVar);
                    generateInitData(dt, newIndent, depth+1, ref);
                    b.a(indent, "}");
                }
                
            }
            
        }
        return b.getContent();
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
