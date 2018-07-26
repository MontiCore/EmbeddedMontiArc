package de.monticore.lang.monticar.generator.roscpp.util;

/**
 * @author Sascha Schneiders
 */
public class VariableType {
    protected String typeNameMontiCar;
    protected String typeNameTargetLanguage;
    protected String includeName = "";

    public VariableType() {

    }

    public VariableType(String typeNameMontiCar, String typeNameTargetLanguage, String includeName) {
        this.typeNameMontiCar = typeNameMontiCar;
        this.typeNameTargetLanguage = typeNameTargetLanguage;
        this.includeName = includeName;
    }

    public String getTypeNameMontiCar() {
        return typeNameMontiCar;
    }

    public void setTypeNameMontiCar(String typeNameMontiCar) {
        this.typeNameMontiCar = typeNameMontiCar;
    }

    public String getIncludeName() {
        return includeName;
    }

    public void setIncludeName(String includeName) {
        this.includeName = includeName;
    }

    public String getTypeNameTargetLanguage() {
        return typeNameTargetLanguage;
    }

    public void setTypeNameTargetLanguage(String typeNameTargetLanguage) {
        this.typeNameTargetLanguage = typeNameTargetLanguage;
    }

    public boolean hasInclude() {
        return !includeName.equals("");
    }

    public void set(VariableType variableType) {
        this.typeNameMontiCar = variableType.typeNameMontiCar;
        this.typeNameTargetLanguage = variableType.typeNameTargetLanguage;
        this.includeName = variableType.includeName;
    }

}
