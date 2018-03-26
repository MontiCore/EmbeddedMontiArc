package de.monticore.lang.monticar.setup;

import de.monticore.lang.monticar.setup.action.Action;
import de.monticore.lang.monticar.setup.action.DownloadAction;
import de.monticore.lang.monticar.setup.action.ExtractionAction;
import de.monticore.lang.monticar.setup.action.SetupAction;
import java.net.URL;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class EmscriptenConfiguration implements Configuration {

  private URL emscripten;
  private Path filePath;
  private Path extractionPath;
  private String osName;

  public EmscriptenConfiguration(URL emscripten, Path filePath, Path extractionPath,
      String osName) {
    this.emscripten = emscripten;
    this.filePath = filePath;
    this.extractionPath = extractionPath;
    this.osName = osName;
  }

  public List<Action> getActions() {
    List<Action> actions = new ArrayList<>();
    actions.add(new DownloadAction(emscripten, filePath));
    actions.add(new ExtractionAction(filePath, extractionPath));
    actions.add(new SetupAction(extractionPath, commands()));
    return actions;
  }

  private List<String[]> commands() {
    List<String[]> commands = new ArrayList<>();
    commands.add(emscriptenUpdate());
    commands.add(emscriptenInstall());
    commands.add(emscriptenActivate());
    return commands;
  }

  private String[] emscriptenUpdate() {
    return arrayof(emsdk(), "update");
  }

  private String[] emscriptenInstall() {
    return arrayof(emsdk(), "install", "latest");
  }

  private String[] emscriptenActivate() {
    return arrayof(emsdk(), "activate", "latest");
  }

  private String emsdk() {
    Path emsdk = Paths.get(osName.startsWith("windows") ?
        ".\\emsdk-master\\emsdk.bat" :
        "./emsdk-master/emsdk");
    return "\"" + extractionPath.resolve(emsdk).toAbsolutePath().normalize().toString() + "\"";
  }

  private String[] arrayof(String... strings) {
    return strings;
  }
}
