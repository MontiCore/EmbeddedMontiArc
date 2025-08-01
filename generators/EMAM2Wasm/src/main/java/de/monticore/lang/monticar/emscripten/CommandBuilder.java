/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import java.util.List;

/**
 * Basic builder interface for command line commands
 */
public interface CommandBuilder {

  /**
   * Returns a list of the command line command in an order defined by the implementing class.
   * Usually the binary is in the first position and further command line switches in the following.
   *
   * @return list of commands
   */
  List<String> toList();

  /**
   * Returns a concatenation of the elements of {@link #toList()} separated by a single space.
   *
   * @return command string
   */
  String toString();
}
