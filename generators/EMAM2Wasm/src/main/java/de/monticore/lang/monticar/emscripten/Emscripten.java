/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

/**
 * This interface defines the command to call emscripten.
 */
public interface Emscripten {

  /**
   * Returns an array possibly containing multiple entries that need to be
   * called in conjunction to invoke emscripten.
   *
   * @param options additional command line options
   * @return command to invoke emscripten
   */
  String[] getCommand(String... options);

}
