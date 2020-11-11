package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.monticore.ast.ASTNode;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.javaclassexpressions._ast.ASTNameExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cpp.FileUtil;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.struct._ast.ASTStruct;
import de.monticore.lang.monticar.struct._ast.ASTStructFieldDefinition;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.types.types._ast.ASTType;

/*
    Generates the CPP files necessary to build the 'DynamicInterface'

    DISCLAIMER: Do not assume the use of the EmbeddedMontiArc framework here is optimal, someone with experience in
    the EMA suite should verify how the different types and symbols are resolved.
*/
public class DynamicInterfaceGenerator {
    
    HashSet<String> cppFileDependencies = new HashSet<>();
    List<FileContent> files = new ArrayList<>();

    String componentName;
    ProgramInterface programInterface;
    String progInterfaceString;
    JsonCommunication dyn;
    TcpCommunication tcp;
    DDCCommunication ddc;

    public DynamicInterfaceGenerator(
        EMAComponentInstanceSymbol componentSymbol, 
        CMakeConfig cmake,
        String outputName,
        boolean genDynamicInterface, 
        boolean genServer, 
        boolean genDDC
    ) throws SerializationException, IOException {
        // Read the ProgramInterface from the model
        resolve(componentSymbol);
        progInterfaceString = Json.toJson(programInterface);

        // Generate the files
        if (genDynamicInterface) {
            dyn = new JsonCommunication(this);
            files.addAll(dyn.generate());
        }
        if (genServer || genDDC) {
            tcp = new TcpCommunication(this);
            files.addAll(tcp.generate(genDDC));
        }
        if (genDDC) {
            ddc = new DDCCommunication(this);
            files.addAll(ddc.generate());
        }
        // Add the common file dependencies
        for (String f : cppFileDependencies) {
            files.add(FileUtil.getResourceAsFile("/template/dynamic_interface/"+f, f));
        }

        if (outputName.length() == 0) outputName = componentSymbol.getName();

        // Generate the CMake
        if (genDynamicInterface) {
            dyn.addCMake(cmake, outputName);
        }
        if (genServer || genDDC) {
            String targetName = outputName+"Adapter";
            HashSet<String> sources = new HashSet<>();
            HashSet<String> libs = new HashSet<>();

            tcp.getSources(sources);
            tcp.getLibs(libs);
            
            if (genDDC) {
                ddc.getSources(sources);
                ddc.getLibs(libs);
            }

            String sourceFiles = "";
            for (String s : sources) sourceFiles += s + ' ';

            // create adapter executable
            cmake.addCMakeCommandEnd("add_executable("+targetName+" "+sourceFiles+")");
            if (!libs.isEmpty()) {
                String libList = "";
                for (String l : libs) libList += l + ' ';
                cmake.addCMakeCommandEnd("target_link_libraries("+targetName+" PUBLIC "+libs+")");
            }
            cmake.addCMakeCommandEnd("target_link_libraries("+targetName+" PUBLIC ${LIBS})");
            cmake.addCMakeCommandEnd("target_compile_features("+targetName+" PUBLIC cxx_std_11)");
            // install adapter
            cmake.addCMakeCommandEnd("install(TARGETS "+targetName+" DESTINATION $ENV{DLL_DIR})");
        }
    }

    public List<FileContent> getFiles() {
        return files;
    }

    public void addCppFileDependency(String name) {
        cppFileDependencies.add(name);
    }


    void resolve(EMAComponentInstanceSymbol componentSymbol){
        programInterface = new ProgramInterface();
        programInterface.name = componentSymbol.getName();
        programInterface.version = "0.0";

        // TODO ignore dynamic ports for now
        this.componentName = GeneralHelperMethods.getTargetLanguageComponentName(componentSymbol.getFullName());
        for ( EMAPortInstanceSymbol port : componentSymbol.getIncomingPortInstances()){
            boolean multipleInputsAllowed = false; // TODO
            boolean optional = false; // TODO
            programInterface.ports.add(new PortInformation(port.getName(), getDataType(port), PortDirection.INPUT, multipleInputsAllowed, optional));
        }
        for ( EMAPortInstanceSymbol port : componentSymbol.getOutgoingPortInstances()){
            boolean multipleInputsAllowed = false; // TODO
            boolean optional = false; // TODO
            programInterface.ports.add(new PortInformation(port.getName(), getDataType(port), PortDirection.OUTPUT, multipleInputsAllowed, optional));
        }

    }

    int evalExpr(ASTExpression expr){
        if (expr.getSymbolOpt().isPresent() && expr.getSymbolOpt().get() instanceof MathExpressionSymbol)
            return Integer.parseInt(((MathExpressionSymbol) expr.getSymbolOpt().get()).getTextualRepresentation());
        else if (expr instanceof ASTNameExpression)
            throw new IllegalArgumentException("Unknown dimension description: "+expr);
        else if (expr instanceof ASTNumberWithUnit){
            return ((ASTNumberWithUnit) expr).getNumber().get().intValue();
        } else if (expr instanceof ASTNumberExpression) {
            return ((ASTNumberExpression) expr).getNumberWithUnit().getNumber().get().intValue();
        } else throw new IllegalArgumentException("Could not evaluate expression: "+expr);
    }

    DataType getVectorType(int size, DataType subtype){
        if (size == 2) {
            return BasicType.VEC2;
        } else if (size == 3) {
            return BasicType.VEC3;
        } else {
            return new VectorType(subtype, size);
        }
    }

    ASTCommonMatrixType getMatrixType(MCTypeReference<? extends MCTypeSymbol> mctype){
        if (mctype.existsReferencedSymbol() && mctype.getReferencedSymbol() instanceof MCASTTypeSymbolReference){
            ASTType astType = ((MCASTTypeSymbolReference) mctype.getReferencedSymbol()).getAstType();
            if (astType instanceof ASTCommonMatrixType) return (ASTCommonMatrixType)astType;
        }
        throw new IllegalArgumentException("Could not get Matrix AST type from mctype");
    }

    DataType getDataType2(ASTType astType){
        // AssignmentType ?
        String typeName = astType.getSymbol().getName();
        if (typeName.equals("Q")){
            return BasicType.Q;
        } else if (typeName.equals("Z")) {
            return BasicType.Z;
        } else if (typeName.equals("N")) {
            return BasicType.N;
        } else if (typeName.equals("N1")) {
            return BasicType.N1;
        } else if (typeName.equals("B") || typeName.equals("Boolean")) {
            return BasicType.BOOLEAN;
        } else if (typeName.equals("C")) {
            return BasicType.C;
        } else if (typeName.equals("CommonMatrixType")) {
            if (!(astType instanceof ASTCommonMatrixType)) throw new IllegalArgumentException("Could not get Matrix AST type from ASTType");
            ASTCommonMatrixType mt = (ASTCommonMatrixType)astType;
            ASTElementType type = mt.getElementType();
            DataType subtype = getDataType2(type);
            if (subtype == null) throw new IllegalArgumentException("Unsupported sub-type for matrix: "+type.getName()); 
            List<ASTExpression> dim = mt.getDimension().getDimensionList();
            if (dim.size() == 1) {
                // Array type
                int size = evalExpr(dim.get(0));
                return getVectorType(size, subtype);
            } else if (dim.size() == 2) {
                // Matrix type => represent with array of arrays
                int rowCount = evalExpr(dim.get(0));
                int colCount = evalExpr(dim.get(1));
                if (rowCount == 1 || colCount == 1){
                    return getVectorType(Math.max(rowCount, colCount), subtype);
                }
                return new MatrixType(subtype, rowCount, colCount);
            } else throw new IllegalArgumentException("Unknown dimensions: "+dim.size());
            //return new VectorType(BasicType.DOUBLE, 128);
        }
        
        Optional<ASTNode> n = astType.getSymbol().getAstNode();
        if (n.isPresent()){
            ASTNode node = n.get();
            if (node instanceof ASTStruct){
                ASTStruct struct = (ASTStruct) node;
                StructType type = new StructType(typeName);
                //System.out.println("Struct fields for "+typeName);
                for (ASTStructFieldDefinition fd : struct.getStructFieldDefinitionList()){
                    //System.out.println(fd.getName());
                    DataType fieldType = getDataType2(fd.getType());
                    type.addField(fd.getName(), fieldType);
                }
                return type;
            } else {
                int a = 0;
            }
        }
        
        System.out.println("Missing DataType conversion: "+typeName);
        return BasicType.EMPTY;
    }

    DataType getBasicType(String typeName){
        if (typeName.equals("Q")){
            return BasicType.Q;
        } else if (typeName.equals("Z")) {
            return BasicType.Z;
        } else if (typeName.equals("N")) {
            return BasicType.N;
        } else if (typeName.equals("N1")) {
            return BasicType.N1;
        } else if (typeName.equals("B") || typeName.equals("Boolean")) {
            return BasicType.BOOLEAN;
        } else if (typeName.equals("C")) {
            return BasicType.C;
        }
        return null;
    }

    // TODO from ASTType ??
    DataType getDataType(EMAPortInstanceSymbol port){
        // AssignmentType ?
        MCTypeReference<? extends MCTypeSymbol> mctype = port.getTypeReference();
        String typeName = mctype.getName();
        if (typeName.equals("Q")){
            return BasicType.Q;
        } else if (typeName.equals("Z")) {
            return BasicType.Z;
        } else if (typeName.equals("N")) {
            return BasicType.N;
        } else if (typeName.equals("N1")) {
            return BasicType.N1;
        } else if (typeName.equals("B") || typeName.equals("Boolean")) {
            return BasicType.BOOLEAN;
        } else if (typeName.equals("C")) {
            return BasicType.C;
        } else if (typeName.equals("CommonMatrixType")) {
            ASTCommonMatrixType mt = getMatrixType(mctype);
            ASTElementType type = mt.getElementType();
            DataType subtype = getBasicType(type.getName());
            if (subtype == null) throw new IllegalArgumentException("Unsupported sub-type for matrix: "+type.getName()); 
            List<ASTExpression> dim = mt.getDimension().getDimensionList();
            if (dim.size() == 1) {
                // Array type
                int size = evalExpr(dim.get(0));
                return getVectorType(size, subtype);
            } else if (dim.size() == 2) {
                // Matrix type => represent with array of arrays
                int rowCount = evalExpr(dim.get(0));
                int colCount = evalExpr(dim.get(1));
                if (rowCount == 1 || colCount == 1){
                    return getVectorType(Math.max(rowCount, colCount), subtype);
                }
                return new MatrixType(subtype, rowCount, colCount);
            } else throw new IllegalArgumentException("Unknown dimensions: "+dim.size());
            //return new VectorType(BasicType.DOUBLE, 128);
        }
        Optional<ASTNode> n = mctype.getReferencedSymbol().getAstNode();
        if (n.isPresent()){
            ASTNode node = n.get();
            if (node instanceof ASTStruct){
                ASTStruct struct = (ASTStruct) node;

                StructType type = new StructType(typeName);

                //System.out.println("Struct fields for "+typeName);
                for (ASTStructFieldDefinition fd : struct.getStructFieldDefinitionList()){
                    //System.out.println(fd.getName());
                    //DataType fieldType = getDataType2(fd.getType());
                    String tn = ((ASTElementType)fd.getType()).getName();
                    DataType fieldType = getBasicType(tn);
                    if (fieldType == null) {
                        throw new IllegalArgumentException("Unsupported sub-type for struct: "+tn); 
                    }
                    type.addField(fd.getName(), fieldType);
                }
                return type;
            } else {
                int a = 0;
            }
        }
        
        System.out.println("Missing DataType conversion: "+typeName);
        return BasicType.EMPTY;
    }

}
