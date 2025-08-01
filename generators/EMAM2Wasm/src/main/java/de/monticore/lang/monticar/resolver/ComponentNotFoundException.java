/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.resolver;

public class ComponentNotFoundException extends RuntimeException {

  public ComponentNotFoundException() {
    super();
  }

  public ComponentNotFoundException(String fullName) {
    super(fullName);
  }
}
