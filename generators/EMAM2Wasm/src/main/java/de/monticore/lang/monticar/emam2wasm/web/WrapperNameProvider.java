/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.web;

import de.monticore.symboltable.Symbol;
import java.nio.file.Path;
import java.nio.file.Paths;

public interface WrapperNameProvider {

  /**
   * Returns the wrapper file name for the supplied model.
   *
   * @param model EmbeddedMontiArc model
   * @return WebAssembly file name
   */
  String getName(Symbol model);

  /**
   * Returns the file extension that should be used when storing the wrapper file.
   *
   * @return wrapper file extension
   */
  String getFileExtension();

  /**
   * Returns a relative path to the file location where the wrapper file should be saved.
   * In this default implementation, the path consists of {@code filename.extension}.
   *
   * @param model EmbeddedMontiArc model
   * @return wrapper file location
   */
  default Path getFilePath(Symbol model) {
    return Paths.get(getName(model) + "." + getFileExtension());
  }
}
