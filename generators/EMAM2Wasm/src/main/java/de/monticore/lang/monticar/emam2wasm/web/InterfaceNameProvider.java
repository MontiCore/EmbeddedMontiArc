/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.web;

import de.monticore.symboltable.Symbol;
import java.nio.file.Path;
import java.nio.file.Paths;

public interface InterfaceNameProvider {

  /**
   * Returns the interface file name for the supplied model.
   *
   * @param model EmbeddedMontiArc model
   * @return WebAssembly file name
   */
  String getName(Symbol model);

  /**
   * Returns the file extension that should be used when storing the interface file.
   *
   * @return interface file extension
   */
  String getFileExtension();

  /**
   * Returns a relative path to the file location where the interface file should be saved.
   * In this default implementation, the path consists of {@code filename.extension}.
   *
   * @param model EmbeddedMontiArc model
   * @return interface file location
   */
  default Path getFilePath(Symbol model) {
    return Paths.get(getName(model) + "." + getFileExtension());
  }
}
