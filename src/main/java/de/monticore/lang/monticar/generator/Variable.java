package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTPrintType;
import de.monticore.lang.monticar.types2._ast.ASTType;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class Variable {
    public static String FORLOOPINFO = "ForLoopVariable";
    public static String VARIABLE = "Variable";
    public static String ORIGINPORT = "OriginPort";
    public static String STATIC = "Static";

    String name = "";
    VariableType type = new VariableType();
    boolean inputVariable = false;
    boolean constantVariable = false;
    boolean parameterVariable = false;
    int arraySize = 1;
    Optional<String> constantValue = Optional.empty();
    List<String> additionalInformation = new ArrayList<>();
    List<String> dimensionalInformation = new ArrayList<>();
    List<String> properties = new ArrayList<>();
    Optional<String> customTypeName = Optional.empty();

    public Variable() {

    }

    public Variable(String name, String additionalInformation) {
        this.name = name;
        this.additionalInformation.add(additionalInformation);
    }

    public Variable(Variable variable) {
        this.type.set(variable.getVariableType());
        this.name = variable.getName();
        this.inputVariable = variable.isInputVariable();
        this.constantVariable = variable.constantVariable;
        this.arraySize = variable.arraySize;
        this.additionalInformation = variable.additionalInformation;
    }

    public List<String> getAdditionalInformation() {
        return additionalInformation;
    }

    public void setAdditionalInformation(List<String> additionalInformation) {
        this.additionalInformation = additionalInformation;
    }

    public void addAdditionalInformation(String information) {
        this.additionalInformation.add(information);
    }

    public boolean hasAdditionalInformation(String information) {
        return additionalInformation.contains(information);
    }

    public boolean isForLoopVariable() {
        return hasAdditionalInformation(Variable.FORLOOPINFO);
    }

    public void setIsConstantVariable(boolean constantVariable) {
        this.constantVariable = constantVariable;
    }

    public void setVariableType(VariableType variableType) {

        this.type.set(variableType);
    }

    public VariableType getVariableType() {
        return type;
    }

    public boolean isInputVariable() {
        return inputVariable;
    }

    public void setInputVariable(boolean isInputVariable) {
        this.inputVariable = isInputVariable;
    }

    public String getName() {
        return name;
    }

    public String getNameTargetLanguageFormat() {
        //TODO refactor this
        return GeneralHelperMethods.getTargetLanguageVariableInstanceName(name);
        //return name.replaceAll("\\[", "_").replaceAll("\\]", "_");
    }

    public void setName(String name) {
        this.name = name;
    }

    public void setTypeNameTargetLanguage(String typeName) {
        type.setTypeNameTargetLanguage(typeName);
    }

    public void setTypeNameMontiCar(String typeName) {
        type.setTypeNameMontiCar(typeName);
    }

    public void setTypeNameMontiCar(ASTType type) {
        if (type instanceof ASTPrintType) {
            ASTPrintType printType = (ASTPrintType) type;
            setTypeNameMontiCar(printType.printType());
            setVariableType(TypeConverter.getVariableTypeForMontiCarTypeName(this.type.getTypeNameMontiCar(), this, type).get());
        } else if (type instanceof ASTElementType) {
            ASTElementType elementType = (ASTElementType) type;
            setTypeNameMontiCar(TypeConverter.getTypeNameMontiCar(elementType));
            setVariableType(TypeConverter.getVariableTypeForMontiCarTypeName(this.type.getTypeNameMontiCar(), this, type).get());
        } else {
            Log.info(type.getClass().getName(), "ASTType:");
            Log.error("Case not handled!");
        }
    }

    public boolean hasInclude() {
        return type.hasInclude();
    }

    public String getIncludeName() {
        return type.getIncludeName();
    }

    public void setArraySize(int size) {
        this.arraySize = size;
    }

    public int getArraySize() {
        return arraySize;
    }

    public boolean isArray() {
        return arraySize > 1;
    }

    public boolean isStaticVariable() {
        return additionalInformation.contains(STATIC);
    }

    public boolean isConstantVariable() {
        return constantVariable;
    }

    public String getConstantValue() {
        return constantValue.get();
    }


    public void setConstantValue(String constantValue) {
        this.constantValue = Optional.of(constantValue);
    }

    public String getNameWithoutArrayNamePart() {
        int indexLast_ = getName().lastIndexOf("]");
        if (indexLast_ != -1) {
            int indexSecondLast_ = getName().lastIndexOf("[", indexLast_ - 1);
            if (indexSecondLast_ != -1)
                return getName().substring(0, indexSecondLast_);
        }
        return name;
    }

    public boolean isCrossComponentVariable() {
        return name.contains(".");
    }

    public String getComponentName() {
        return name.split("\\.")[0];
    }

    public void addDimensionalInformation(String dimension) {
        Log.info(name + " " + dimension, "Added DimInfo:");
        dimensionalInformation.add(dimension);
    }

    public List<String> getDimensionalInformation() {
        return dimensionalInformation;
    }

    public void setDimensionalInformation(List<String> dimensionalInformation) {
        this.dimensionalInformation = dimensionalInformation;
    }

    public void setDimensionalInformation(String dimension1, String dimension2) {
        this.dimensionalInformation.clear();
        addDimensionalInformation(dimension1);
        addDimensionalInformation(dimension2);
    }

    public int howManyDimensions() {
        return dimensionalInformation.size();
    }

    public boolean isParameterVariable() {
        return parameterVariable;
    }

    public List<String> getProperties() {
        return properties;
    }

    public void addProperties(List<String> properties) {
        this.properties.addAll(properties);
    }

    public void addProperty(String property) {
        this.properties.add(property);
    }

    public void setIsParameterVariable(boolean parameterVariable) {
        this.parameterVariable = parameterVariable;
    }

    public static String convertToVariableName(String fullName) {
        return fullName.replaceAll("\\.", "_");
    }
}
