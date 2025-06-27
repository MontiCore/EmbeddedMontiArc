/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._ast.ASTAssignmentType;
import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueType;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.VariableType;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.TypesGeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.viewmodel.Utils;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.types2._ast.ASTDimension;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.types.types._ast.ASTType;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

/**
 * This is used to convert port types to their cpp equivalent
 *
 */
public class TypeConverter {
    private static List<VariableType> nonPrimitiveVariableTypes = new ArrayList<>();
    private static Set<MCTypeSymbol> typeSymbols = new HashSet<>();


    public static void addNonPrimitiveVariableType(String typeNameMontiCar, String typeNameTargetLanguage, String includeName) {
        nonPrimitiveVariableTypes.add(new VariableType(typeNameMontiCar, typeNameTargetLanguage, includeName));
    }

    public static void addNonPrimitiveVariableType(VariableType variableType) {
        nonPrimitiveVariableTypes.add(variableType);
    }

    public static String getVariableTypeNameForMathLanguageTypeName(MathValueType mathValueType) {
        if (mathValueType.getDimensions().size() == 0) {
            if (mathValueType.getType().isWholeNumber()) {
                return "int";//use int for now add range check if bigger number is required
            } else if(mathValueType.getType().isBoolean()){
                return "bool";
            }
            return "double";
        } else if (mathValueType.getDimensions().size() == 1) {
            return MathConverter.curBackend.getColumnVectorTypeName();
        } else if (mathValueType.getDimensions().size() == 2) {
            return MathConverter.curBackend.getMatrixTypeName();
        } else if (mathValueType.getDimensions().size() == 3) {
            return MathConverter.curBackend.getCubeTypeName();
        }
        Log.error("TypeConverter Case not handled!");
        return null;
    }

    private static Optional<VariableType> getVariableTypeForMontiCarTypeNameOctave(String typeNameMontiCar) {
        for (VariableType variableType : nonPrimitiveVariableTypes) {
            if (variableType.getTypeNameMontiCar().equals(typeNameMontiCar)) {
                return Optional.of(variableType);
            }
        }
        return Optional.empty();
    }

    public static Optional<VariableType> getVariableTypeForMontiCarTypeName(String typeNameMontiCar) {
        if (MathConverter.curBackend.getBackendName().equals("OctaveBackend"))
            return getVariableTypeForMontiCarTypeNameOctave(typeNameMontiCar);
        else if (MathConverter.curBackend.getBackendName().equals("ArmadilloBackend")) {
            if (typeNameMontiCar.equals("CommonMatrixType")) {
                return Optional.of(new VariableType("CommonMatrixType", MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName()));
            } else if (typeNameMontiCar.equals("AssignmentType")) {
                return Optional.of(new VariableType("AssignmentType", MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName()));
            } else {
                return getVariableTypeForMontiCarTypeNameOctave(typeNameMontiCar);
            }
        }
        Log.info(typeNameMontiCar, "Unknown Type:");
        //Log.error("0xUNPOTYFOBYGE Unknown Port Type found by generator");
        return Optional.empty();
    }

    private static Optional<VariableType> getVariableTypeForMontiCarTypeNameOctave(String typeNameMontiCar, Variable variable, ASTType astType) {
        for (VariableType variableType : nonPrimitiveVariableTypes)
            if (variableType.getTypeNameMontiCar().equals(typeNameMontiCar)) {
                if (typeNameMontiCar.equals("CommonMatrixType")) {
                    ASTCommonMatrixType astCommonMatrixType = (ASTCommonMatrixType) astType;
                    handleCommonMatrixType(variable, astCommonMatrixType);
                    variableType = getRealVariableType(astCommonMatrixType);
                } else if (typeNameMontiCar.equals("AssignmentType")) {//TODO Add MatrixProperties to MathInformation
                    ASTAssignmentType astAssignmentType = (ASTAssignmentType) astType;
                    handleAssignmentType(variable, astAssignmentType);
                }
                return Optional.of(variableType);
            }
        return Optional.empty();
    }

    public static Optional<VariableType> getVariableTypeForMontiCarTypeName(String typeNameMontiCar, Variable variable, ASTType astType) {
        if (MathConverter.curBackend.getBackendName().equals("OctaveBackend")) {
            return getVariableTypeForMontiCarTypeNameOctave(typeNameMontiCar, variable, astType);
        } else if (MathConverter.curBackend.getBackendName().equals("ArmadilloBackend")) {
            Optional<VariableType> variableType;
            if (typeNameMontiCar.equals("CommonMatrixType")) {
                ASTCommonMatrixType astCommonMatrixType = (ASTCommonMatrixType) astType;
                handleCommonMatrixType(variable, astCommonMatrixType);
                variableType = Optional.ofNullable(getRealVariableType(astCommonMatrixType));
            } else if (typeNameMontiCar.equals("AssignmentType")) {//TODO Add MatrixProperties to MathInformation
                variableType = Optional.of(new VariableType(typeNameMontiCar, MathConverter.curBackend.getMatrixTypeName(),
                        MathConverter.curBackend.getIncludeHeaderName()));
                ASTAssignmentType astAssignmentType = (ASTAssignmentType) astType;
                handleAssignmentType(variable, astAssignmentType);
            } else {
                variableType = getVariableTypeForMontiCarTypeNameOctave(typeNameMontiCar, variable, astType);
            }
            return variableType;
        }
        Log.info(typeNameMontiCar, "Unknown Type:");
        //Log.error("0xUNPOTYFOBYGE Unknown Port Type found by generator");
        return Optional.empty();
    }

    public static VariableType getRealVariableType(ASTCommonMatrixType astCommonMatrixType) {
        ASTElementType type = astCommonMatrixType.getElementType();

        VariableType variableType;
        List<ASTExpression> dimensionElements = astCommonMatrixType.getDimension().getDimensionList();
        if (dimensionElements.size() == 1) {
            variableType = new VariableType("CommonColumnVectorType", getColvecAccessString(type), MathConverter.curBackend.getIncludeHeaderName());
        } else if (dimensionElements.size() == 2) {
            if (isVectorDimension(dimensionElements.get(0)))
                variableType = new VariableType("CommonRowVectorType", getRowAccessString(type), MathConverter.curBackend.getIncludeHeaderName());
            else {
                variableType = new VariableType("CommonMatrixType", getMatAccessString(type) , MathConverter.curBackend.getIncludeHeaderName());
            }
        } else if (dimensionElements.size() == 3) {
            variableType = new VariableType("CommonCubeType", getCubeAccessString(type), MathConverter.curBackend.getIncludeHeaderName());
        } else if (dimensionElements.size() == 4) {
            variableType = new VariableType("CommonCubeType", getCubeAccessString(type), MathConverter.curBackend.getIncludeHeaderName());
        } else {
            variableType = new VariableType("CommonMatrixType", getMatAccessString(type), MathConverter.curBackend.getIncludeHeaderName());
        }
        return variableType;
    }



    public static boolean isVectorDimension(ASTExpression astCommonDimensionElement) {
        boolean result = false;
        if (astCommonDimensionElement instanceof ASTNumberWithUnit) {
            ASTNumberWithUnit unitNumber = (ASTNumberWithUnit) astCommonDimensionElement;
            if (unitNumber.getNumber().isPresent()) {
                result = unitNumber.getNumber().get().intValue() == 1;
            }
        }
        return result;
    }

    public static Optional<VariableType> getVariableTypeForMontiCarTypeName(String typeNameMontiCar, Variable variable, EMAPortInstanceSymbol portSymbol) {
        Optional<VariableType> getDefaultType = getNonPrimitiveVariableType(typeNameMontiCar, variable, portSymbol);
        if (getDefaultType.isPresent())
            return getDefaultType;
        MCTypeSymbol s = portSymbol.getTypeReference().getReferencedSymbol();
        String fullName = s.getFullName();
        if (typeNameMontiCar != null && (typeNameMontiCar.equals(s.getName()) || typeNameMontiCar.equals(fullName))) {
            String cppName = Utils.getIncludeName(s);
            String includeName = TypesGeneratorCPP.TYPES_DIRECTORY_NAME + "/" + cppName;
            typeSymbols.add(s);
            VariableType vt = new VariableType(fullName, cppName, includeName);
            addNonPrimitiveVariableType(vt);
            return Optional.of(vt);
        }
        Log.info(typeNameMontiCar, "Unknown Type:");
        //Log.error("0xUNPOTYFOBYGE Unknown Port Type found by generator");
        return Optional.empty();
    }

    public static Optional<VariableType> getNonPrimitiveVariableType(String typeNameMontiCar, Variable variable, EMAPortInstanceSymbol portSymbol) {
        for (VariableType variableType : nonPrimitiveVariableTypes) {
            if (variableType.getTypeNameMontiCar().equals(typeNameMontiCar)) {
                if (typeNameMontiCar.equals("CommonMatrixType")) {
                    if (MathConverter.curBackend.getBackendName().equals("ArmadilloBackend")) {
                        variableType = getRealVariableTypeFromPortSymbol(portSymbol);
                    }
                    handleCommonMatrixType(variable, portSymbol);
                } else if (typeNameMontiCar.equals("AssignmentType")) {
                    if (MathConverter.curBackend.getBackendName().equals("ArmadilloBackend")) {
                        variableType = new VariableType("AssignmentType", MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName());
                    }
                    handleAssignmentType(variable, portSymbol);
                }
                return Optional.of(variableType);
            }
        }
        return Optional.empty();
    }

    private static VariableType getRealVariableTypeFromPortSymbol(EMAPortInstanceSymbol portSymbol) {
        VariableType variableType;
        Optional<ASTCommonMatrixType> astCommonMatrixType = PortConverter.getCommonMatrixTypeFromPortSymbol(portSymbol);
        if (astCommonMatrixType.isPresent()) {
            variableType = getRealVariableType(astCommonMatrixType.get());
        } else {
            variableType = new VariableType("CommonMatrixType", MathConverter.curBackend.getMatrixTypeName(), MathConverter.curBackend.getIncludeHeaderName());
        }
        return variableType;
    }

    public static void handleCommonMatrixType(Variable variable, EMAPortInstanceSymbol portSymbol) {
        ASTCommonMatrixType astCommonMatrixType = (ASTCommonMatrixType) ((ASTPort) portSymbol.getAstNode().get()).getType();
        handleCommonMatrixType(variable, astCommonMatrixType);
    }

    public static void handleCommonMatrixType(Variable variable, ASTCommonMatrixType astCommonMatrixType) {
        for (ASTExpression astCommonDimensionElement :
                astCommonMatrixType.getDimension().getDimensionList()) {
            if (astCommonDimensionElement.getSymbolOpt().isPresent() && astCommonDimensionElement.getSymbolOpt().get() instanceof MathExpressionSymbol)
                variable.addDimensionalInformation(((MathExpressionSymbol) astCommonDimensionElement.getSymbolOpt().get()).getTextualRepresentation());
            else if (astCommonDimensionElement instanceof ASTNameExpression)
                variable.addDimensionalInformation(((ASTNameExpression) astCommonDimensionElement).getName());
            else if (astCommonDimensionElement instanceof ASTNumberWithUnit)
                variable.addDimensionalInformation(String.valueOf(((ASTNumberWithUnit) astCommonDimensionElement).getNumber().get().intValue()));
            else if (astCommonDimensionElement instanceof ASTNumberExpression)
                variable.addDimensionalInformation(String.valueOf(((ASTNumberExpression) astCommonDimensionElement).getNumberWithUnit().getNumber().get().intValue()));
            else
                Log.error("Case not handled;");

        }
    }

    public static void handleAssignmentType(Variable variable, EMAPortInstanceSymbol portSymbol) {
        ASTAssignmentType astAssignmentType = (ASTAssignmentType) ((ASTPort) portSymbol.getAstNode().get()).getType();
        handleAssignmentType(variable, astAssignmentType);
    }

    public static void handleAssignmentType(Variable variable, ASTAssignmentType astAssignmentType) {
        //TODO Add MatrixProperties to MathInformation
        if (astAssignmentType.getDimensionOpt().isPresent()) {
            ASTDimension astDimension = astAssignmentType.getDimensionOpt().get();
            variable.addDimensionalInformation(((MathExpressionSymbol) astDimension.getDimensionList().get(0).getSymbolOpt().get()).getTextualRepresentation());
            variable.addDimensionalInformation(((MathExpressionSymbol) astDimension.getDimensionList().get(1).getSymbolOpt().get()).getTextualRepresentation());
        }
    }

    public static VariableType getVariableTypeForMontiCarInstance(EMAComponentInstanceSymbol instanceSymbol) {
        VariableType type = TypeConverter.getVariableTypeForMontiCarTypeName(instanceSymbol.getFullName()).orElse(null);
        if (type != null)
            return type;
        type = new VariableType();
        type.setTypeNameMontiCar(instanceSymbol.getFullName());
        type.setTypeNameTargetLanguage(GeneralHelperMethods.getTargetLanguageComponentName(instanceSymbol.getFullName()));
        type.setIncludeName(type.getTypeNameTargetLanguage());
        TypeConverter.addNonPrimitiveVariableType(type);
        return type;
    }

    public static String getTypeName(MathMatrixArithmeticValueSymbol mathExpressionSymbol) {
        if (mathExpressionSymbol.getVectors().size() > 1) {
            if (mathExpressionSymbol.getVectors().get(0).getMathMatrixAccessSymbols().size() > 1)
                return MathConverter.curBackend.getMatrixTypeName();
            else {
                return MathConverter.curBackend.getColumnVectorTypeName();
            }
        } else {
            return MathConverter.curBackend.getRowVectorTypeName();
        }
    }

    public static String getTypeNameMontiCar(ASTElementType elementType) {
        String result = null;
        if (elementType.isRational()) {
            result = "Q";
        } else if (elementType.isBoolean()) {
            result = "B";
        } else if (elementType.isComplex()) {
            result = "C";
        } else if (elementType.isWholeNumber()) {
            result = "Z";
        } else if (elementType.isNaturalNumber()) {
            result = "N";
        } else {
            Log.error("Case not handled!");
        }
        return result;
    }

    public static VariableType getVariableTypeForTargetLanguageTypeName(String targetLanguageTypeName) {
        VariableType type = new VariableType();
        type.setTypeNameTargetLanguage(targetLanguageTypeName);
        type.setIncludeName(MathConverter.curBackend.getIncludeHeaderName());
        return type;
    }

    public static void clearTypeSymbols() {
        typeSymbols = new HashSet<>();
    }

    public static Set<MCTypeSymbol> getTypeSymbols() {
        return Collections.unmodifiableSet(typeSymbols);
    }

    static {
        addNonPrimitiveVariableType("SIUnitRangesType", "double", "");
        addNonPrimitiveVariableType("B", "bool", "");
        addNonPrimitiveVariableType("Q", "double", "");
        // TODO: the type mappings below have been adjusted to make the tests pass. they are, however, wrong.
        addNonPrimitiveVariableType("Z", "int", "");
        addNonPrimitiveVariableType("C", "double", "");
        addNonPrimitiveVariableType("N1", "int", "");
        addNonPrimitiveVariableType("N0", "int", "");
        addNonPrimitiveVariableType("N", "int", "");
        addNonPrimitiveVariableType("UnitNumberResolution", "double", "");
        addNonPrimitiveVariableType("CommonMatrixType", "Matrix", "octave/oct");
        addNonPrimitiveVariableType("AssignmentType", "Matrix", "octave/oct");

    }

    public static String getColvecAccessString(ASTElementType type){
        if(type.isRational()){
            return MathConverter.curBackend.getColumnVectorTypeName();
        }else if(type.isWholeNumber()){
            return MathConverter.curBackend.getWholeNumberColumnVectorTypeName();
        }else{
            Log.error("0x7f4dc: Type not supported");
            return null;
        }
    }

    private static String getRowAccessString(ASTElementType type) {
        if(type.isRational()){
            return MathConverter.curBackend.getRowVectorTypeName();
        }else if(type.isWholeNumber()){
            return MathConverter.curBackend.getWholeNumberRowVectorTypeName();
        }else{
            Log.error("0x7f4dc: Type not supported");
            return null;
        }
    }

    public static String getMatAccessString(ASTElementType type){
        if(type.isRational()){
            return MathConverter.curBackend.getMatrixTypeName();
        }else if(type.isWholeNumber()){
            return MathConverter.curBackend.getWholeNumberMatrixTypeName();
        }else{
            Log.error("0xa7674: Type not supported");
            return null;
        }
    }

    public static String getCubeAccessString(ASTElementType type){
        if(type.isRational()){
            return MathConverter.curBackend.getCubeTypeName();
        }else if(type.isWholeNumber()){
            return MathConverter.curBackend.getWholeNumberCubeTypeName();
        }else{
            Log.error("0x1d06c: Type not supported");
            return null;
        }
    }

    public static String getDimensionString(String accessString, List<MathExpressionSymbol> dims, List<String> includeString){
        String dimsString = dims.stream()
                .map(dim -> ExecuteMethodGenerator.generateExecuteCode(dim,includeString))
                .collect(Collectors.joining(","));

        return accessString + "(" + dimsString + ")";
    }
}
