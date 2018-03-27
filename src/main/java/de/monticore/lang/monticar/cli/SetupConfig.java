package de.monticore.lang.monticar.cli;

import de.monticore.lang.monticar.emscripten.Shell;
import de.monticore.lang.monticar.setup.BasicConfiguration;
import de.monticore.lang.monticar.setup.action.DownloadAction;
import de.monticore.lang.monticar.setup.action.ExtractionAction;
import de.monticore.lang.monticar.setup.action.SetupAction;
import java.net.URL;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;

@Configuration
@ComponentScan("de.monticore.lang.monticar.setup")
public class SetupConfig {

  private static final String DOCKER_INSTALL_COMMAND =
      "docker run -dit --name emscripten -v $(pwd):/src trzeci/emscripten:sdk-incoming-64bit bash";
  private static final Path EMSDK_WIN = Paths.get("./emsdk-master/emsdk.bat");
  private static final Path EMSDK_LINUX = Paths.get("./emsdk-master/emsdk");

  @Value("https://github.com/juj/emsdk/archive/master.zip")
  private URL emscriptenURL;

  @Value("#{systemProperties['user.home']}/emscripten/emscripten.zip")
  private Path emscriptenDownloadPath;

  @Value("#{systemProperties['user.home']}/emscripten/")
  private Path emscriptenExtractionPath;

  @Value("#{systemProperties['os.name'].toLowerCase()}")
  private String osName;

  @Value("https://github.com/conradsnicta/armadillo-code/archive/8.400.x.zip")
  private URL armadilloURL;

  @Value("#{systemProperties['user.home']}/armadillo/armadillo.zip")
  private Path armadilloDownloadPath;

  @Value("#{systemProperties['user.home']}/armadillo/")
  private Path armadilloExtractionPath;

  @Bean
  public de.monticore.lang.monticar.setup.Configuration armadilloConfiguration() {
    return new BasicConfiguration(
        new DownloadAction(armadilloURL, armadilloDownloadPath),
        new ExtractionAction(armadilloDownloadPath, armadilloExtractionPath));
  }

  @Bean
  public de.monticore.lang.monticar.setup.Configuration emscriptenConfiguration() {
    if (isWindows()) {
      return emscriptenWindowsConfig();
    }
    return emscriptenLinuxConfig();
  }

  private BasicConfiguration emscriptenLinuxConfig() {
    return new BasicConfiguration(new SetupAction(emscriptenLinuxCommands()));
  }

  private BasicConfiguration emscriptenWindowsConfig() {
    return new BasicConfiguration(
        new DownloadAction(emscriptenURL, emscriptenDownloadPath),
        new ExtractionAction(emscriptenDownloadPath, emscriptenExtractionPath),
        new SetupAction(emscriptenExtractionPath, emscriptenWindowsCommands()));
  }

  private List<String[]> emscriptenLinuxCommands() {
    Shell shell = Shell.BASH;
    List<String[]> commands = new ArrayList<>();
    commands.add(new String[]{
        shell.getShellCommand(),
        shell.getExecuteSwitch(),
        DOCKER_INSTALL_COMMAND});
    return commands;
  }

  private List<String[]> emscriptenWindowsCommands() {
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
    if (isWindows()) {
      String emsdk = emscriptenExtractionPath.resolve(EMSDK_WIN)
          .toAbsolutePath().normalize().toString();
      return "\"" + emsdk + "\"";
    }
    return emscriptenExtractionPath.resolve(EMSDK_LINUX).toAbsolutePath().normalize().toString();
  }

  private boolean isWindows() {
    return osName.startsWith("windows");
  }

  private String[] arrayof(String... strings) {
    return strings;
  }


}
