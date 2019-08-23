/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.wasm;

public class WasmCompilerException extends RuntimeException {

  public WasmCompilerException(String message) {
    super(message);
  }

  public WasmCompilerException(Throwable cause) {
    super(cause);
  }

  public WasmCompilerException(String message, Throwable cause) {
    super(message, cause);
  }

}
