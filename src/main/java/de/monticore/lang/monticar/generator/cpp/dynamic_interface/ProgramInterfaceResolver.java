package de.monticore.lang.monticar.generator.cpp.dynamic_interface;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortType;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.ast.ASTNode;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.javaclassexpressions._ast.ASTNameExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.struct._ast.ASTStruct;
import de.monticore.lang.monticar.struct._ast.ASTStructFieldDefinition;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.types.types._ast.ASTType;


/**
 * This class reads the main (root) EMA component and fills a ProgramInterface structure corresponding to the ports of the component.
 * 
 * DISCLAIMER: Do not assume the use of the EmbeddedMontiArc framework here is optimal, someone with experience in
 * the EMA suite should verify how the different types and symbols are resolved.
 * 
 * TODO maybe this should be read from the blueprint structure?
 */
public class ProgramInterfaceResolver {
    public static String SOCK_IN_SUFFIX = "_socket_in";
    public static String SOCK_OUT_SUFFIX = "_socket_out";
    public static String SOCK_BC_IN_SUFFIX = "_socket_bc_in";
    public static String SOCK_BC_OUT_SUFFIX = "_socket_bc_out";

    ProgramInterface programInterface;
    String componentName;
    String progInterfaceString;
    
    void resolve(EMAComponentInstanceSymbol componentSymbol) throws SerializationException {
        programInterface = new ProgramInterface();
        programInterface.name = componentSymbol.getName();
        programInterface.version = ProgramInterface.CURRENT_VERSION;

        // TODO ignore dynamic ports for now
        this.componentName = GeneralHelperMethods.getTargetLanguageComponentName(componentSymbol.getFullName());

        for ( EMAPortInstanceSymbol port : componentSymbol.getIncomingPortInstances()) {
            String name = port.getName();
            boolean isArray = isPortArray(port.getName());
            if (isArray) name = getPortArrayName(port.getName());

            if (isSocketPort(name)) {
                if (!isSocketInput(name)) throw new IllegalArgumentException("Port '"+name+"' must be declared as output with this suffix.");
                resolveSocket(port, name, isArray, true);
            } else {
                if (isArray) throw new IllegalArgumentException("ArrayPorts not implemented");

                boolean multipleInputsAllowed = false; // TODO
                boolean optional = false; // TODO
                addPortInformation(PortInformation.newInputDataPort(name, getDataType(port), multipleInputsAllowed, optional));
            }

        }

        for ( EMAPortInstanceSymbol port : componentSymbol.getOutgoingPortInstances()) {
            String name = port.getName();
            boolean isArray = isPortArray(port.getName());
            if (isArray) name = getPortArrayName(port.getName());

            if (isSocketPort(name)) {
                if (isSocketInput(name)) throw new IllegalArgumentException("Port '"+name+"' must be declared as input with this suffix.");
                resolveSocket(port, name, isArray, false);
            } else {
                if (isArray) throw new IllegalArgumentException("ArrayPorts not implemented");

                boolean optional = false; // TODO
                addPortInformation(PortInformation.newOutputDataPort(name, getDataType(port), optional));
            }
        }

        // For sockets: Check array lengths and convert data type to simple packet
        for (PortInformation port : programInterface.ports) {
            if (port.port_type != PortType.SOCKET) continue;
            port.data_type = new SimplePacketType(port.data_type);

            SocketInfo sockInf = sockByName.get(port.name);
            sockInf.array_length = -1;
            if (sockInf.is_bc) {
                if (port.isInput()) {
                    if (arraySizes.containsKey(port.name + SOCK_BC_IN_SUFFIX)){
                        sockInf.array_length = arraySizes.get(port.name + SOCK_BC_IN_SUFFIX);
                    } else throw new IllegalArgumentException("Socket-port '"+port.name + SOCK_BC_IN_SUFFIX+"' is not a port-array.");
                }
                if (port.isOutput()) {
                    if (arraySizes.containsKey(port.name + SOCK_BC_OUT_SUFFIX)) {
                        throw new IllegalArgumentException("Socket-port '"+port.name + SOCK_BC_OUT_SUFFIX+"' cannot be a  port-array (must be size of one).");
                    }
                }
            } else {
                if (port.isInput()) {
                    if (arraySizes.containsKey(port.name + SOCK_IN_SUFFIX)){
                        sockInf.array_length = arraySizes.get(port.name + SOCK_IN_SUFFIX);
                    } else throw new IllegalArgumentException("Socket-port '"+port.name + SOCK_IN_SUFFIX+"' is not a port-array.");
                }
                if (port.isOutput()) {
                    if (arraySizes.containsKey(port.name + SOCK_OUT_SUFFIX)){
                        int size = arraySizes.get(port.name + SOCK_OUT_SUFFIX);
                        if (sockInf.array_length == -1) sockInf.array_length = size;
                        else if (sockInf.array_length != size) throw new IllegalArgumentException("PortArray size mismatch between input and output for port '"+port.name+"'.");
                    } else throw new IllegalArgumentException("Socket-port '"+port.name + SOCK_OUT_SUFFIX+"' is not a port-array.");
                }
            }
        }

        progInterfaceString = Json.toJson(programInterface);
    }


    
    void resolveSocket(EMAPortInstanceSymbol port, String name, boolean isArray, boolean isInput) {
        String targetName = getSocketTargetName(name);

        if (isArray) {
            trackPortArraySize(name, getPortArrayIndex(port.getName()));
            if (!isFirstEncounterAndSet(name, arrayPorts)) return;
        }

        PortInformation portInf = portByName.get(targetName);
        if (portInf == null) {
            addPortInformation(PortInformation.newSocketPort(targetName, getDataType(port), isInput, !isInput));
            sockByName.put(targetName, new SocketInfo(!isInput, isSocketBroadcast(name), name));
        } else {
            // Another socket already maps to the target name
            // Cannot be a DATA port
            if (portInf.port_type == PortType.DATA) throw new IllegalArgumentException("Port Name Clash: EMA Port '"+portInf.name+"' and EMA socket port '"+name+"' map to the same port name ('"+targetName+"').");
            // DataType must match
            DataType thisDataType = getDataType(port);
            if (!portInf.data_type.equals(thisDataType)) throw new IllegalArgumentException("Socket port DataType mismatch between input and output for '"+portInf.name+"': "+portInf.data_type+" and "+thisDataType);
            // 'Broadcast' must match
            SocketInfo sockInf = sockByName.get(targetName);
            if (sockInf.is_bc != isSocketBroadcast(name)) throw new IllegalArgumentException("Mixed BROADCAST usage of socket-port '"+targetName+"'.");

            if (isInput) {
                sockInf.input_name = name;
            } else {
                sockInf.output_name = name;
            }

            portInf.direction = PortDirection.IO;
            portInf.addTag("network");
        }
    }


    public static class SocketInfo {
        boolean is_bc;
        int array_length;
        String input_name;
        String output_name;
        SocketInfo(boolean output, boolean bc, String originalName) {
            this.is_bc = bc;
            if (output) {
                output_name = originalName;
            } else {
                input_name = originalName;
            }
        }
    }
    HashMap<String, Integer> arraySizes = new HashMap<>();
    HashMap<String, PortInformation> portByName = new HashMap<>();
    HashMap<String, SocketInfo> sockByName = new HashMap<>();
    HashSet<String> arrayPorts = new HashSet<>();

    SocketInfo getSocketInfo(PortInformation portInfo) {
        return sockByName.get(portInfo.name);
    }

    static boolean isPortArray(String portName) {
        return portName.charAt(portName.length()-1) == ']';
    }

    static int getPortArrayIndex(String portName) {
        int p = portName.length()-2;
        while (p >= 0 && portName.charAt(p) != '[') --p;
        return Integer.parseInt(portName.substring(p+1, portName.length()-1));
    }
    
    static String getPortArrayName(String portName) {
        int p = portName.length()-2;
        while (p >= 0 && portName.charAt(p) != '[') --p;
        return portName.substring(0, p);
    }

    void trackPortArraySize(String name, int index) {
        Integer size = arraySizes.get(name);
        if (size == null || size < index) arraySizes.put(name, index);
    }

    static boolean isFirstEncounterAndSet(String name, HashSet<String> encountered) {
        if (encountered.contains(name)) return false;
        encountered.add(name);
        return true;
    }

    static boolean isSocketPort(String name) {
        return name.endsWith(SOCK_BC_IN_SUFFIX) || name.endsWith(SOCK_IN_SUFFIX) || name.endsWith(SOCK_BC_OUT_SUFFIX) || name.endsWith(SOCK_OUT_SUFFIX);
    }
    static boolean isSocketInput(String name) {
        return name.endsWith(SOCK_BC_IN_SUFFIX) || name.endsWith(SOCK_IN_SUFFIX);
    }
    static boolean isSocketBroadcast(String name) {
        return name.endsWith(SOCK_BC_IN_SUFFIX) || name.endsWith(SOCK_BC_OUT_SUFFIX);
    }

    void addPortInformation(PortInformation portInf) {
        // Add portInf and check there is no name clash
        if (portByName.containsKey(portInf.name)) {
            throw new IllegalArgumentException("Port Name Clash: Multiple EMA ports map to the same name: '"+portInf.name+"' (is one a socket port with suffix?)");
        }
        portByName.put(portInf.name, portInf);
        programInterface.ports.add(portInf);
    }

    static String removeSuffix(String str, String suffix) {
        return str.substring(0, str.length()-suffix.length());
    }

    static String getSocketTargetName(String name) {
        if (name.endsWith(SOCK_BC_IN_SUFFIX)) return removeSuffix(name, SOCK_BC_IN_SUFFIX);
        if (name.endsWith(SOCK_IN_SUFFIX)) return removeSuffix(name, SOCK_IN_SUFFIX);
        if (name.endsWith(SOCK_BC_OUT_SUFFIX)) return removeSuffix(name, SOCK_BC_OUT_SUFFIX);
        if (name.endsWith(SOCK_OUT_SUFFIX)) return removeSuffix(name, SOCK_OUT_SUFFIX);
        return name;
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
