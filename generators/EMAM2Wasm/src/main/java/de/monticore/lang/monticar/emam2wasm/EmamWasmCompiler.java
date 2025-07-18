/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import java.nio.file.Path;

/**
 * Compiles EmbeddedMontiArc (emam) models to WebAssembly (wasm).
 */
public interface EmamWasmCompiler {

  /**
   * Compiles the supplied model to WebAssembly code and returns the path to
   * the compiled file. <p>
   * When using emscripten and the output file is specified (e.g. .html or
   * .js), the returned path should point to this output file
   *
   * @param model EmbeddedMontiArc model to compile to WebAssembly
   * @return path to compiled output file
   */
  Path emam2wasm(ExpandedComponentInstanceSymbol model);
}
