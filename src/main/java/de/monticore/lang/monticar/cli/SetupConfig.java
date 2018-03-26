package de.monticore.lang.monticar.cli;

import de.monticore.lang.monticar.setup.ArmadilloConfiguration;
import de.monticore.lang.monticar.setup.EmscriptenConfiguration;
import java.net.URL;
import java.nio.file.Path;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;

@Configuration
@ComponentScan("de.monticore.lang.monticar.setup")
public class SetupConfig {

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
  public EmscriptenConfiguration emscriptenConfiguration() {
    return new EmscriptenConfiguration(emscriptenURL, emscriptenDownloadPath,
        emscriptenExtractionPath, osName);
  }

  @Bean
  public ArmadilloConfiguration armadilloConfiguration() {
    return new ArmadilloConfiguration(armadilloURL, armadilloDownloadPath,
        armadilloExtractionPath);
  }
}
