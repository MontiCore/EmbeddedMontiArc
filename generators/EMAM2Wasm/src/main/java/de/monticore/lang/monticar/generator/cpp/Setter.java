/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

public class Setter extends Method {

  private String variableName;
  private String variableType;
  private String parameterName;

  public Setter(Datatype datatype, Type type, String methodName, String variableName,
      String variableType, String parameterName) {
    super(datatype, type, methodName);
    this.variableName = variableName;
    this.variableType = variableType;
    this.parameterName = parameterName;
  }

  public String getVariableName() {
    return variableName;
  }

  public void setVariableName(String variableName) {
    this.variableName = variableName;
  }

  public String getVariableType() {
    return variableType;
  }

  public void setVariableType(String variableType) {
    this.variableType = variableType;
  }

  public String getParameterName() {
    return parameterName;
  }

  public void setParameterName(String parameterName) {
    this.parameterName = parameterName;
  }
}
