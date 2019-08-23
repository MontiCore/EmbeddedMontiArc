/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.cpp;

import de.monticore.symboltable.Symbol;
import java.nio.file.Path;

/**
 * This interface is used to compile MontiArc models to C++ code.
 *
 * @param <T> Any subclass  of {@link Symbol} that can represent a MontiArc model
 */
public interface CppCompiler<T extends Symbol> {

  /**
   * Compiles the supplied MontiArc model to C++ code. The returned path points
   * to the main C++ file that can be used to instantiate the model.
   *
   * @param symbol MontiArc model to compile
   * @param outDir Directory, that the C++ code is compiled to. This directory
   * will be created if it does not yet exist.
   * @param mainFileName Path to main C++ file
   */
  Path compile(T symbol, Path outDir, String mainFileName) throws CppCompilerException;

}
