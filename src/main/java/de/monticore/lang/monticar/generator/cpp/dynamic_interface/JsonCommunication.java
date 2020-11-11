package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;
import de.rwth.montisim.commons.utils.json.SerializationException;

public class JsonCommunication {
    static final int BASE_INDENT = 2;

    //public static void generateCMake(CMakeConfig cmake,)

    final DynamicInterfaceGenerator gen;

    FileBuilder b = new FileBuilder();
    
    JsonCommunication(DynamicInterfaceGenerator gen) {
        this.gen = gen;
    }

    public List<FileContent> generate() throws SerializationException {
        List<FileContent> files = new ArrayList<>();
        List<String> getPortCases = new ArrayList<>();
        List<String> setPortCases = new ArrayList<>();

        int i = 0;
        for (PortInformation portInfo : gen.programInterface.ports){
            if (portInfo.direction == PortDirection.INPUT){
                setPortCases.add(generateSetPortCase(i, portInfo));
            } else {
                getPortCases.add(generateGetPortCase(i, portInfo));
            }
            ++i;
        }

        HashMap<String, Object> templateData = new HashMap<>();
        templateData.put("mainModelName", gen.componentName);
        templateData.put("interfaceDescription", gen.progInterfaceString);
        templateData.put("getPortCases", getPortCases);
        templateData.put("setPortCases", setPortCases);

        files.add(new FileContent(AllTemplates.generate(AllTemplates.DYNAMIC_INTERFACE_H, templateData), "dynamic_interface.h"));
        files.add(new FileContent(AllTemplates.generate(AllTemplates.DYNAMIC_INTERFACE_CPP, templateData), "dynamic_interface.cpp"));

        gen.addCppFileDependency("json.h");
        gen.addCppFileDependency("json.cpp");
        gen.addCppFileDependency("printf.h");
        gen.addCppFileDependency("printf.cpp");
        gen.addCppFileDependency("utils.h");

        return files;
    }
    

    
	public void addCMake(CMakeConfig cmake, String outputName) {
        // create shared lib
        cmake.addCMakeCommandEnd("add_library("+outputName+" SHARED dynamic_interface.cpp json.cpp printf.cpp)");
        cmake.addCMakeCommandEnd("target_include_directories("+outputName+" PUBLIC ${INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})");
        cmake.addCMakeCommandEnd("target_link_libraries("+outputName+" PUBLIC ${LIBS} -static-libgcc -static-libstdc++)");
        cmake.addCMakeCommandEnd("set_target_properties("+outputName+" PROPERTIES LINKER_LANGUAGE CXX POSITION_INDEPENDENT_CODE ON)");
        cmake.addCMakeCommandEnd("target_compile_features("+outputName+" PUBLIC cxx_std_11)");
        cmake.addCMakeCommandEnd("IF (WIN32)");
        cmake.addCMakeCommandEnd("  set_target_properties("+outputName+" PROPERTIES PREFIX \"\")");
        cmake.addCMakeCommandEnd("ENDIF()");
        // install shared lib
        cmake.addCMakeCommandEnd("install(TARGETS "+outputName+" DESTINATION $ENV{DLL_DIR})");
        //cmake.addCMakeCommandEnd("export(TARGETS "+outputName+" FILE "+outputName+".cmake)");
	}
    
    public String generateSetPortCase(int id, PortInformation portInfo){
        b.init();

        b.a(BASE_INDENT, "case %d: { // %s", id, portInfo.name);
        generateSetter(portInfo.type, BASE_INDENT+1, 1, "program_instance."+portInfo.name);
        b.a(BASE_INDENT, "} break;");
        
        return b.getContent();
    }

    public String generateGetPortCase(int id, PortInformation portInfo){
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
                b.a(indent, "%s = traverser.get_double();", varReference);
                    break;
                case Z:
                b.a(indent, "%s = (int32_t) traverser.get_long();", varReference);
                    break;
                case N: {
                    String varName = String.format("res%d", depth);
                    b.a(indent, "auto %s = traverser.get_long();",           varName);
                    b.a(indent, "if (%s < 0) { %s = 0; } //TODO as error?",  varName, varName);
                    b.a(indent, "%s = %s;",                                  varReference, varName);
                } break;
                case N1: {
                    String varName = String.format("res%d", depth);
                    b.a(indent, "auto %s = traverser.get_long();",           varName);
                    b.a(indent, "if (%s < 1) { %s = 0; } //TODO as error?",  varName, varName);
                    b.a(indent, "%s = %s;",                                  varReference, varName);
                } break;
                case C: {
                    String i = String.format("i%d", depth);
                    b.a(indent, "int %s = 0;",                                   i);
                    b.a(indent, "for (auto t : traverser.stream_array()) {"      );
                    b.a(indent, "    if (%s == 0) %s = traverser.get_double();", i, varReference);
                    b.a(indent, "    else if (%s == 1) {} // TODO UPDATE\n",     i); // TODO handle correct Complex native type
                    b.a(indent, "    else {} // TODO error?"                     );
                    b.a(indent, "    ++%s;",                                     i);
                    b.a(indent, "} if (%s < 2) {} // TODO error?",               i);
                } break;
                case BOOLEAN:
                b.a(indent, "%s = traverser.get_bool();", varReference);
                    break;
                case EMPTY:
                    break;
                case VEC2: {
                    String i = String.format("i%d", depth);
                    b.a(indent, "int %s = 0;",                                       i);
                    b.a(indent, "for (auto t : traverser.stream_array()) {"          );
                    b.a(indent, "    if (%s < 2) %s(%s) = traverser.get_double();",  i, varReference, i);
                    b.a(indent, "    else {} // TODO error?"                         );
                    b.a(indent, "    ++%s;",                                         i);
                    b.a(indent, "} if (%s < 2) {} // TODO error?",                   i);
                } break;
                case VEC3: {
                    String i = String.format("i%d", depth);
                    b.a(indent, "int %s = 0;",                                       i);
                    b.a(indent, "for (auto t : traverser.stream_array()) {"          );
                    b.a(indent, "    if (%s < 3) %s(%s) = traverser.get_double();",  i, varReference, i);
                    b.a(indent, "    else {} // TODO error?"                         );
                    b.a(indent, "    ++%s;",                                         i);
                    b.a(indent, "} if (%s < 3) {} // TODO error?",                   i);
                } break;
                default:
                    throw new IllegalArgumentException("Missing case");
            }
        } else if (type instanceof VectorType) {
            VectorType vt = (VectorType)type;
            int newIndent = indent + 1;
            int size = vt.getSize();
            String i = String.format("i%d", depth);
            String ref = String.format("res%d", depth);

            b.a(indent, "int %s = 0;",                                   i);
            b.a(indent, "for (auto t : traverser.stream_array()) {"      );
            b.a(indent, "    if (%s >= %d) { break; /*TODO error?*/}",   i, size);
            b.a(indent, "    auto &%s = %s(%s);",                        ref, varReference, i);
            generateSetter(vt.getBaseType(), newIndent, depth+1, ref);
            b.a(indent, "    ++%s;",                                     i);
            b.a(indent, "} if (%s < %d) {} // TODO error?",              i, size);
        } else if (type instanceof MatrixType) {
            MatrixType vt = (MatrixType)type;
            int newIndent = indent + 2;
            int rows = vt.getRowCount();
            int cols = vt.getColumnCount();
            String i = String.format("i%d", depth);
            String j = String.format("j%d", depth);
            String ref = String.format("res%d", depth);

            
            b.a(indent, "int %s = 0;",                                       i);
            b.a(indent, "for (auto t : traverser.stream_array()) {"          );
            b.a(indent, "    if (%s >= %d) { break; /*TODO error?*/}",       i, rows);
            b.a(indent, "    int %s = 0;",                                   j);
            b.a(indent, "    for (auto u : traverser.stream_array()) {"      );
            b.a(indent, "        if (%s >= %d) { break; /*TODO error?*/}",   j, cols);
            b.a(indent, "        auto &%s = %s(%s, %s);",                    ref, varReference, i, j);
            generateSetter(vt.getBaseType(), newIndent, depth+1, ref);
            b.a(indent, "        ++%s;",                                     j);
            b.a(indent, "    } if (%s < %d) {} // TODO error?",              j, cols);
            b.a(indent, "    ++%s;",                                         i);
            b.a(indent, "} if (%s < %d) {} // TODO error?",                  i, cols);

        } else if (type instanceof DynVectorType) {
            System.out.println("Unimplemented: DynVectorType set_port case.");
            //throw new IllegalArgumentException("Unimplemented");
        } else if (type instanceof StructType) {
            StructType st = (StructType)type;
            int newIndent = indent + 1;
            int count = st.getFieldCount();
            String array_stream = String.format("array_stream%d", depth);
            String end = String.format("end%d", depth);
            String it = String.format("it%d", depth);

            
            b.a(indent, "auto %s = traverser.stream_array();",   array_stream);
            b.a(indent, "auto %s = %s.end();",                   end, array_stream);
            b.a(indent, "auto %s = %s.begin();", it,             array_stream);

            for (int i = 0; i < count; ++i){
                String fieldName = st.getFieldName(i);
            
                b.a(indent, "if (%s != %s) {",           it, end);
                generateSetter(st.getFieldType(i), newIndent, depth+1, String.format("%s.%s", varReference, fieldName));
                b.a(indent, "    %s++;",                 it);
                b.a(indent, "} else {/*TODO error?*/}"   );
            }

            b.a(indent, "if (%s != %s) {/*TODO error?*/}", it, end);

        } else if (type instanceof EnumType) {
            EnumType et = (EnumType)type;
            int variantCount = et.getVariantCount();
            String elseS = "";
            String variant = String.format("variant%d", depth);

            
            b.a(indent, "auto %s = traverser.get_string();", variant);

            for (int i = 0; i < variantCount; ++i){
                String value = et.getVariant(i);
                
                b.a(indent, "%sif (%s.equals(\"%s\")) {",    elseS, variant, value);
                b.a(indent, "    %s= %s;",                   varReference, value);
                b.a(indent, "}"                              );

                elseS = "else ";
            }

            b.a(indent, "else {/*TODO error?*/}");

        } else {
            throw new IllegalArgumentException("Missing case");
        }
    }

    private void generateGetter(DataType type, int indent, int depth, String varReference) {

        if (type instanceof BasicType){
            BasicType t = (BasicType) type;
            switch (t.getType()){
                case Q:
                case Z:
                case N:
                case N1:
                case BOOLEAN:
                b.a(indent, "writer.write_value(%s);", varReference);
                    break;
                case EMPTY:
                    break;
                case C:
                b.a(indent, "writer.start_array();"      );
                b.a(indent, "writer.write_value(%s);",   varReference);
                b.a(indent, "writer.write_value(0); // TODO handle correct Complex native type");
                b.a(indent, "writer.end_array();"        );
                    break;
                case VEC2:
                b.a(indent, "writer.start_array();"          );
                b.a(indent, "writer.write_value(%s(0));",    varReference);
                b.a(indent, "writer.write_value(%s(1));",    varReference);
                b.a(indent, "writer.end_array();"            );
                    break;
                case VEC3:
                b.a(indent, "writer.start_array();"          );
                b.a(indent, "writer.write_value(%s(0));",    varReference);
                b.a(indent, "writer.write_value(%s(1));",    varReference);
                b.a(indent, "writer.write_value(%s(2));",    varReference);
                b.a(indent, "writer.end_array();"            );
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

            b.a(indent, "writer.start_array();"              );
            b.a(indent, "for (int %s=0; %s < %d; ++%s) {",   i, i, size, i);
            b.a(indent, "    auto &%s = %s(%s);",            ref, varReference, i);
            generateGetter(vt.getBaseType(), newIndent, depth+1, ref);
            b.a(indent, "}"                                  );
            b.a(indent, "writer.end_array();"                );

        } else if (type instanceof MatrixType) {
            MatrixType vt = (MatrixType)type;
            int newIndent = indent + 2;
            int rows = vt.getRowCount();
            int cols = vt.getColumnCount();
            String i = String.format("i%d", depth);
            String j = String.format("j%d", depth);
            String ref = String.format("res%d", depth);

            
            b.a(indent, "writer.start_array();"                  );
            b.a(indent, "for (int %s=0; %s < %d; ++%s) {",       i, i, rows, i);
            b.a(indent, "    writer.start_array();"              );
            b.a(indent, "    for (int %s=0; %s < %d; ++%s) {",   j, j, cols, j);
            b.a(indent, "        auto &%s = %s(%s, %s);",        ref, varReference, i, j);
            generateGetter(vt.getBaseType(), newIndent, depth+1, ref);
            b.a(indent, "    }"                                  );
            b.a(indent, "    writer.end_array();"                );
            b.a(indent, "}"                                      );
            b.a(indent, "writer.end_array();"                    );

        }else if (type instanceof DynVectorType) {
            System.out.println("Unimplemented: DynVectorType get_port case.");
            //throw new IllegalArgumentException("Unimplemented");
        } else if (type instanceof StructType) {
            StructType st = (StructType)type;
            int count = st.getFieldCount();

            
            b.a(indent, "writer.start_array();");
            for (int i = 0; i < count; ++i){
                String fieldName = st.getFieldName(i);
                generateGetter(st.getFieldType(i), indent, depth+1, String.format("%s.%s", varReference, fieldName));
            }
            b.a(indent, "writer.end_array();");

        } else if (type instanceof EnumType) {
            EnumType et = (EnumType)type;
            int variantCount = et.getVariantCount();
            String variant = String.format("variant%d", depth);

            
            b.a(indent, "switch(%s) {", varReference);
            for (int i = 0; i < variantCount; ++i){
                String value = et.getVariant(i);
                b.a(indent, "case %s: writer.write_value(\"%s\"); break;", value, value);
            }
            b.a(indent, "}");
        } else {
            throw new IllegalArgumentException("Missing case");
        }
    }


}
