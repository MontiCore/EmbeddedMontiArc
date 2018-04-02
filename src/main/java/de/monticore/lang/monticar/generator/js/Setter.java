package de.monticore.lang.monticar.generator.js;

public class Setter {

  private String methodName;
  private String parameterName;
  private String delegateMethodName;
  private int[] dimension;
  private String unit;
  private String lowerBound;
  private String upperBound;

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

  public int[] getDimension() {
    return dimension;
  }

  public void setDimension(int[] dimension) {
    this.dimension = dimension;
  }

  public String getUnit() {
    return unit;
  }

  public void setUnit(String unit) {
    this.unit = unit;
  }

  public String getLowerBound() {
    return lowerBound;
  }

  public void setLowerBound(String lowerBound) {
    this.lowerBound = lowerBound;
  }

  public String getUpperBound() {
    return upperBound;
  }

  public void setUpperBound(String upperBound) {
    this.upperBound = upperBound;
  }
}
