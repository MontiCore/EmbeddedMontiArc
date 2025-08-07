/* (c) https://github.com/MontiCore/monticore */
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
  public String[] getCommand(String... options) {
    String[] command = new String[options.length + 1];
    command[0] = emscripten();
    System.arraycopy(options, 0, command, 1, options.length);
    return command;
  }

  private String emscripten() {
    return emscripten.toAbsolutePath().normalize().toString();
  }
}
