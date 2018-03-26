package de.monticore.lang.monticar.emscripten;

/**
 * This interface defines the command to call emscripten.
 */
public interface Emscripten {

  /**
   * Returns an array possibly containing multiple entries that need to be
   * called in conjunction to invoke emscripten.<p>
   * In general, command line parameters for emscripten can be simply
   * concatenated to this array.
   *
   * @return command to invoke emscripten
   */
  String[] getCommand();

}
