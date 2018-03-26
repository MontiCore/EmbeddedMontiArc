package de.monticore.lang.monticar.emscripten;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;

import java.nio.file.Path;

/**
 * This class represents an emscripten binary that can be simply called by
 * calling the absolute path to this binary.
 */
public class EmscriptenBinary implements Emscripten {

  private Path emscripten;

  public EmscriptenBinary(Path emscripten) {
    this.emscripten = requiresNotNull(emscripten);
  }

  @Override
  public String[] getCommand() {
    return new String[]{emscripten()};
  }

  private String emscripten() {
    return emscripten.toAbsolutePath().normalize().toString();
  }
}
