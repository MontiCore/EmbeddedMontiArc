/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

public abstract class Method {

  private Datatype datatype;
  private Type type;
  private String methodName;

  public Method(Datatype datatype, Type type, String methodName) {
    this.datatype = datatype;
    this.type = type;
    this.methodName = methodName;
  }

  public Type getType() {
    return type;
  }

  public void setType(Type type) {
    this.type = type;
  }

  public String getMethodName() {
    return methodName;
  }

  public void setMethodName(String methodName) {
    this.methodName = methodName;
  }

  public Datatype getDatatype() {
    return datatype;
  }

  public void setDatatype(Datatype datatype) {
    this.datatype = datatype;
  }

  public enum Datatype {
    PRIMITIVE,
    MATRIX
  }

  public enum Type {
    SCALAR,
    ARRAY
  }
}
