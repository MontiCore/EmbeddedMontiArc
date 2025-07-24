/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

public enum Shell {
  CMD("cmd", "/C"),
  POWERSHELL("powershell", "-command"),
  BASH("/bin/bash", "-c"),
  SH("/bin/sh", "-c");

  private final String shellCommand;
  private final String executeSwitch;

  Shell(String shellCommand, String executeSwitch) {
    this.shellCommand = shellCommand;
    this.executeSwitch = executeSwitch;
  }

  public String getShellCommand() {
    return shellCommand;
  }

  public String getExecuteSwitch() {
    return executeSwitch;
  }
}
