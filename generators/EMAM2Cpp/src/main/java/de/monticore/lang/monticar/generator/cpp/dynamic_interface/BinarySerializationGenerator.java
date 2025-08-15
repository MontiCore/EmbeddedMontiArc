package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

import de.monticore.lang.monticar.generator.cpp.dynamic_interface.ProgramInterfaceResolver.SocketInfo;
import de.rwth.montisim.commons.dynamicinterface.*;

public class BinarySerializationGenerator extends SerializationGenerator {
    

    BinarySerializationGenerator(AdapterGenerator gen) {
        super(gen);
    }

    String getType() {
        return "Binary";
    }

    boolean isJson() {
        return false;
    }

    void generateSocketSetter(PortInformation portInfo) {
        SocketInfo sockInf = gen.interfaceResolver.getSocketInfo(portInfo);
        int indent = BASE_INDENT+1;
        // std::string ip = br.read_str();
        b.a(indent, "auto ip = reader.read_str();");
        b.a(indent, "auto id = get_socket_id(ip, %d);", sockInf.array_length);
        b.a(indent, "if (id < 0) return;");
        b.a(indent, "auto &target = program_instance.%s[id];", sockInf.input_name);
        generateSetter(((SimplePacketType) portInfo.data_type).getPayloadType(), BASE_INDENT+1, 1, "target");

    }

    void generateSocketGetter(PortInformation portInfo) {
        SocketInfo sockInf = gen.interfaceResolver.getSocketInfo(portInfo);
        int indent = BASE_INDENT+1;

        b.a(indent, "static int id = 0;");
        b.a(indent, "if (id >= %d) {", sockInf.is_bc ? 1 : sockInf.array_length);
        b.a(indent, "    id = 0;");
        b.a(indent, "    return;");
        b.a(indent, "}");

        if (sockInf.is_bc) {
            b.a(indent, "writer.write_str(N_TO_N_BROADCAST_ADDR); // Write address");
            b.a(indent, "auto &target = program_instance.%s;", sockInf.output_name);
        } else {
            b.a(indent, "writer.write_str(N_TO_N_PREFIX + std::to_string(id+1)); // Write address");
            b.a(indent, "auto &target = program_instance.%s[id];", sockInf.output_name);
        }

        generateGetter(((SimplePacketType) portInfo.data_type).getPayloadType(), BASE_INDENT+1, 1, "target");

        b.a(indent, "++id;");
    }

    protected void generateSetter(DataType type, int indent, int depth, String varReference){

        if (type instanceof BasicType){
            BasicType t = (BasicType) type;
            switch (t.getType()){
                case Q:
                b.a(indent, "%s = reader.read_f64();", varReference);
                    break;
                case Z:
                case N:
                case N1: // TOTO check range for N1 & N ?
                b.a(indent, "%s = (int32_t) reader.read_u32();", varReference);
                    break;
                case C:
                b.a(indent, "%s = reader.read_f64();", varReference);
                b.a(indent, "%s = reader.read_f64();", varReference); // TODO correct complex number native case
                throw new IllegalStateException("Unimplemented");
                case BOOLEAN:
                b.a(indent, "%s = reader.read_u8() != 0;", varReference);
                    break;
                case EMPTY:
                    break;
                case VEC2:
                b.a(indent, "%s(0) = reader.read_f64();", varReference);
                b.a(indent, "%s(1) = reader.read_f64();", varReference);
                    break;
                case VEC3:
                b.a(indent, "%s(0) = reader.read_f64();", varReference);
                b.a(indent, "%s(1) = reader.read_f64();", varReference);
                b.a(indent, "%s(2) = reader.read_f64();", varReference);
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
            b.a(indent, "auto %s = reader.read_u16();", sizeVar);
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

            b.a(indent, "%s = static_cast<%s>(reader.read_u8());", et.getName());
        } else {
            throw new IllegalArgumentException("Missing case");
        }

    }

    protected void generateGetter(DataType type, int indent, int depth, String varReference) {

        if (type instanceof BasicType){
            BasicType t = (BasicType) type;
            switch (t.getType()){
                case Q:
                b.a(indent, "writer.write_f64(%s);", varReference);
                break;
                case Z:
                case N:
                case N1:
                b.a(indent, "writer.write_u32(%s);", varReference);
                break;
                case BOOLEAN:
                b.a(indent, "writer.write_u8(%s ? 1 : 0);", varReference);
                    break;
                case EMPTY:
                    break;
                case C:
                b.a(indent, "writer.write_f64(%s);", varReference);
                b.a(indent, "writer.write_f64(0);");
                throw new IllegalStateException("Unimplemented: Missing Proper native Complex Type");
                case VEC2:
                b.a(indent, "writer.write_f64(%s(0));", varReference);
                b.a(indent, "writer.write_f64(%s(1));", varReference);
                    break;
                case VEC3:
                b.a(indent, "writer.write_f64(%s(0));", varReference);
                b.a(indent, "writer.write_f64(%s(1));", varReference);
                b.a(indent, "writer.write_f64(%s(2));", varReference);
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

            b.a(indent, "writer.write_u16(%d);", size);
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

            b.a(indent, "writer.write_u8(static_cast<uint8_t>(%s));", varReference);
        } else {
            throw new IllegalArgumentException("Missing case");
        }
    }

}
