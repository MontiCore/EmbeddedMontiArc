/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.symboltable.Symbol;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * This interface offers methods providing a C++ file name depending on a
 * {@link ExpandedComponentInstanceSymbol}.
 */
public interface CppNameProvider {

  /**
   * Returns the C++ file name for the supplied model.
   *
   * @param model EmbeddedMontiArc model
   * @return C++ file name
   */
  String getName(Symbol model);

  /**
   * Returns the file extension that should be used when storing the C++ file.
   *
   * @return C++ file extension
   */
  String getFileExtension();

  /**
   * Returns a relative path to the file location where the C++ file should be saved.
   * In this default implementation, the path consists of {@code filename.extension}.
   *
   * @param model EmbeddedMontiArc model
   * @return C++ file location
   */
  default Path getFilePath(Symbol model) {
    return Paths.get(getName(model) + "." + getFileExtension());
  }

}
