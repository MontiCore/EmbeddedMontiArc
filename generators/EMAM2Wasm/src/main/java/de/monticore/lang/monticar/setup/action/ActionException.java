/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.setup.action;

public class ActionException extends RuntimeException {

  public ActionException() {
  }

  public ActionException(String message) {
    super(message);
  }

  public ActionException(String message, Throwable cause) {
    super(message, cause);
  }
}
