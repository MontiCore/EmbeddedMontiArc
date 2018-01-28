package de.monticore.lang.monticar.emam2wasm;

public class CppCompilerException extends RuntimeException {

  public CppCompilerException(String message) {
    super(message);
  }

  public CppCompilerException(Throwable cause) {
    super(cause);
  }

  public CppCompilerException(String message, Throwable cause) {
    super(message, cause);
  }
}
