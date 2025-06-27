/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;
import de.monticore.lang.monticar.printtype._ast.ASTPrintType;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.types.types._ast.ASTType;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 */
public class Variable {
    public static final String INCOMING = "Incoming";
    public static final String OUTGOING = "outgoing";
    public static final String COMPONENT = "component";
    public static String FORLOOPINFO = "ForLoopVariable";
    public static String VARIABLE = "Variable";
    public static String ORIGINPORT = "OriginPort";
    public static String STATIC = "Static";
    public static String FROMMATH = "fromMath";
    public static String CROSSCOMPONENT = "CrossComponent";

    String name = "";
    VariableType type = new VariableType();
    boolean inputVariable = false;
    boolean constantVariable = false;
    boolean parameterVariable = false;
    int arraySize = 1;
    boolean isArray = false;


    boolean isPublic = true;
    boolean isDynamic = false;


    Optional<String> constantValue = Optional.empty();
    List<String> additionalInformation = new ArrayList<>();
    List<String> dimensionalInformation = new ArrayList<>();
    List<String> properties = new ArrayList<>();
    Optional<String> customTypeName = Optional.empty();

    public void setArray(boolean array) {
        isArray = array;
    }

    public Variable() {

    }

    public Variable(String name, String additionalInformation) {
        this.name = name;
        this.additionalInformation.add(additionalInformation);
        updateArrayStatus();
    }

    public Variable(Variable variable) {
        this.type.set(variable.getVariableType());
        this.name = variable.getName();
        this.inputVariable = variable.isInputVariable();
        this.constantVariable = variable.constantVariable;
        this.arraySize = variable.arraySize;
        this.additionalInformation = variable.additionalInformation;
        this.isDynamic = variable.isArray;
        updateArrayStatus();
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
        updateArrayStatus();
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
            Log.debug("setTypeNameMontiCar","Case not handled");
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
        updateArrayStatus();
    }

    public int getArraySize() {
        return arraySize;
    }

    public boolean isArray() {
        return isArray;
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
        return additionalInformation.contains(CROSSCOMPONENT);
    }

    public String getComponentName() {
        String[] split = name.split("\\.");
        if (split.length >= 2)
            return split[split.length - 2];
        return "";
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

    private void updateArrayStatus() {
        if ((this.name.contains("[") && this.name.contains("]")) || arraySize > 1) {
            isArray = true;
            Log.debug("set isArray to true of v: " + getName(), "VARIABLE");
        } else {
            //keep array status until explicitly changed
        }
    }

    public boolean isDynamic() {
        return isDynamic;
    }

    public void setDynamic(boolean dynamic) {
        isDynamic = dynamic;
    }

    public boolean isPublic() {
        return isPublic;
    }

    public void setPublic(boolean aPublic) {
        isPublic = aPublic;
    }
}
