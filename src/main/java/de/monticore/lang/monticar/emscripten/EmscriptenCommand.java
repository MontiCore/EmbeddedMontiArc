package de.monticore.lang.monticar.emscripten;

/**
 * This class represents an emscripten command that needs to invoke a shell
 * first and passes the actual emscripten call to this shell.
 */
public class EmscriptenCommand implements Emscripten {

  private Shell shell;
  private String emscriptenCall;

  public EmscriptenCommand(Shell shell, String emscriptenCall) {
    this.shell = shell;
    this.emscriptenCall = emscriptenCall;
  }

  @Override
  public String[] getCommand() {
    return new String[]{shell.getShellCommand(), shell.getExecuteSwitch(), emscriptenCall};
  }
}
