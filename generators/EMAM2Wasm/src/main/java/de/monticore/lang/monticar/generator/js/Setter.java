/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.js;

public class Setter {

  private String methodName;
  private String parameterName;
  private String delegateMethodName;
  private String[] dimension;
  private String lowerBoundValue;
  private String lowerBoundUnit;
  private String upperBoundValue;
  private String upperBoundUnit;

  public String getMethodName() {
    return methodName;
  }

  public void setMethodName(String methodName) {
    this.methodName = methodName;
  }

  public String getParameterName() {
    return parameterName;
  }

  public void setParameterName(String parameterName) {
    this.parameterName = parameterName;
  }

  public String getDelegateMethodName() {
    return delegateMethodName;
  }

  public void setDelegateMethodName(String delegateMethodName) {
    this.delegateMethodName = delegateMethodName;
  }

  public String[] getDimension() {
    return dimension;
  }

  public void setDimension(String[] dimension) {
    this.dimension = dimension;
  }

  public String getLowerBoundValue() {
    return lowerBoundValue;
  }

  public void setLowerBoundValue(String lowerBoundValue) {
    this.lowerBoundValue = lowerBoundValue;
  }

  public String getLowerBoundUnit() {
    return lowerBoundUnit;
  }

  public void setLowerBoundUnit(String lowerBoundUnit) {
    this.lowerBoundUnit = lowerBoundUnit;
  }

  public String getUpperBoundValue() {
    return upperBoundValue;
  }

  public void setUpperBoundValue(String upperBoundValue) {
    this.upperBoundValue = upperBoundValue;
  }

  public String getUpperBoundUnit() {
    return upperBoundUnit;
  }

  public void setUpperBoundUnit(String upperBoundUnit) {
    this.upperBoundUnit = upperBoundUnit;
  }
}
