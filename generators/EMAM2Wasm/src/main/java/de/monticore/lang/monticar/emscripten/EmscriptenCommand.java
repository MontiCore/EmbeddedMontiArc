/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;
import static de.monticore.lang.monticar.contract.StringPrecondition.requiresNotEmpty;

import java.util.Arrays;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * This class represents an emscripten command that needs to invoke a shell
 * first and passes the actual emscripten call to this shell.
 */
public class EmscriptenCommand implements Emscripten {

  private Shell shell;
  private String emscriptenCall;

  public EmscriptenCommand(Shell shell, String emscriptenCall) {
    this.shell = requiresNotNull(shell);
    this.emscriptenCall = requiresNotEmpty(emscriptenCall);
  }

  @Override
  public String[] getCommand(String... options) {
    String emscripten = Stream.concat(Stream.of(emscriptenCall), Arrays.stream(options))
        .collect(Collectors.joining(" "));
    return new String[]{
        shell.getShellCommand(),
        shell.getExecuteSwitch(),
        emscripten
    };
  }
}
