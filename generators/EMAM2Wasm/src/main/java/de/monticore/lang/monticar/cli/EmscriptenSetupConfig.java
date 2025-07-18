/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cli;

import de.monticore.lang.monticar.emscripten.Shell;
import de.monticore.lang.monticar.setup.BasicConfiguration;
import de.monticore.lang.monticar.setup.action.DownloadAction;
import de.monticore.lang.monticar.setup.action.ExtractionAction;
import de.monticore.lang.monticar.setup.action.SetupAction;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.net.URL;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Conditional;
import org.springframework.context.annotation.Configuration;
import org.springframework.context.annotation.Profile;

@Profile("setup")
@Configuration
@ComponentScan("de.monticore.lang.monticar.setup")
public class EmscriptenSetupConfig {

  @Value("${emscripten.setup.docker:docker run -dit --name emscripten -v $(pwd):/src trzeci/emscripten:sdk-incoming-64bit bash}")
  private String dockerInstall;

  @Value("${emscripten.setup.url:https://github.com/juj/emsdk/archive/master.zip}")
  private URL emscriptenURL;

  @Value("#{systemProperties['emscripten.setup.download'] ?: systemProperties['user.home'].concat('/emscripten/emscripten.zip')}")
  private Path emscriptenDownloadPath;

  @Value("#{systemProperties['emscripten.setup.extract'] ?: systemProperties['user.home'].concat('/emscripten/')}")
  private Path emscriptenExtractionPath;

  @Value("${emscripten.execute.docker:docker exec -it emscripten}")
  private String emscriptenDockerCommand;

  @Value("${emscripten.execute.binary.branch:emsdk-master}")
  private String emscriptenBranch;

  @Value("${emscripten.execute.binary.dir:emscripten}")
  private String emscriptenDirectory;

  @Value("${emscripten.emsdk.win:emsdk.bat}")
  private Path emsdk;

  @Value("${emscripten.execute.binary.name:emcc.bat}")
  private String emcc;

  @Bean
  @Conditional(WindowsCondition.class)
  public de.monticore.lang.monticar.setup.Configuration emscriptenConfigurationWindows() {
    return emscriptenWindowsConfig();
  }

  @Bean
  @Conditional(OtherOSCondition.class)
  public de.monticore.lang.monticar.setup.Configuration emscriptenConfigurationOther() {
    return emscriptenLinuxConfig();
  }

  @Bean
  @Conditional(WindowsCondition.class)
  public Supplier<String> emscriptenCommandWindowsSupplier() {
    return this::emscriptenCommandWindows;
  }

  @Bean
  @Conditional(OtherOSCondition.class)
  public Supplier<String> emscriptenCommandOtherSupplier() {
    return this::emscriptenCommandOther;
  }

  public String emscriptenCommandWindows() {
    //full path looks something like emscripten/emsdk-master/emscripten/1.37.36/emcc.bat
    Path emscriptenDir = emscriptenExtractionPath.resolve(emscriptenBranch)
        .resolve(emscriptenDirectory);
    Path binaryDir;
    try {
      binaryDir = findSubDirectory(emscriptenDir);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    return binaryDir.resolve(emcc).toAbsolutePath().normalize().toString();
  }

  public String emscriptenCommandOther() {
    return emscriptenDockerCommand;
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
        dockerInstall});
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
    String emsdk = emscriptenExtractionPath.resolve(emscriptenBranch).resolve(this.emsdk)
        .toAbsolutePath().normalize().toString();
    return "\"" + emsdk + "\"";
  }

  private Path findSubDirectory(Path path) throws IOException {
    try (DirectoryStream<Path> stream = Files.newDirectoryStream(path)) {
      for (Path p : stream) {
        if (Files.isDirectory(p)) {
          return p;
        }
      }
    }
    return null;
  }

  private String[] arrayof(String... strings) {
    return strings;
  }

}
