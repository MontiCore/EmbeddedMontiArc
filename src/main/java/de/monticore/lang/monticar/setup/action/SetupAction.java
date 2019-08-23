/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.setup.action;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

public class SetupAction implements Action {

  private Path workdir;
  private List<String[]> commands;
  private ProcessBuilder pb;

  public SetupAction(List<String[]> commands) {
    this(null, commands);
  }

  public SetupAction(Path workdir, List<String[]> commands) {
    this.workdir = workdir;
    this.commands = commands;
    this.pb = new ProcessBuilder();
  }

  public Path getWorkdir() {
    return workdir;
  }

  public List<String[]> getCommands() {
    return commands;
  }

  @Override
  public void execute() throws ActionException {
    pb.inheritIO();
    if (workdir != null) {
      pb.directory(workdir.toFile());
    }
    for (String[] command : commands) {
      pb.command(command);

      int returnCode;
      try {
        returnCode = pb.start().waitFor();
      } catch (IOException | InterruptedException e) {
        throw new ActionException("Executing command " + Arrays.toString(command) + " failed", e);
      }
      if (returnCode != 0) {
        throw new ActionException("Errors occurred when running " + Arrays.toString(command));
      }
    }
  }
}
