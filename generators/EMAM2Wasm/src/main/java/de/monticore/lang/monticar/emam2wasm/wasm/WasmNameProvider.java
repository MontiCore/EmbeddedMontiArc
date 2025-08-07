/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.wasm;

import java.nio.file.Path;
import java.nio.file.Paths;

public interface WasmNameProvider {

  /**
   * Returns the WebAssembly file name for the supplied C++ file.
   *
   * @param cppFile C++ file
   * @return WebAssembly file name
   */
  String getName(Path cppFile);

  /**
   * Returns the file extension that should be used when storing the WebAssembly file.
   *
   * @return WebAssembly file extension
   */
  String getFileExtension();

  /**
   * Returns a relative path to the file location where the WebAssembly file should be saved.
   * In this default implementation, the path consists of {@code filename.extension}.
   *
   * @param cppFile C++ file
   * @return WebAssembly file location
   */
  default Path getFilePath(Path cppFile) {
    return Paths.get(getName(cppFile) + "." + getFileExtension());
  }

}
