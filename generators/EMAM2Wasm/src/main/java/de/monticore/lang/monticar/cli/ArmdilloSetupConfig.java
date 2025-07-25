/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cli;

import de.monticore.lang.monticar.setup.BasicConfiguration;
import de.monticore.lang.monticar.setup.action.DownloadAction;
import de.monticore.lang.monticar.setup.action.ExtractionAction;
import java.net.URL;
import java.nio.file.Path;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;
import org.springframework.context.annotation.Profile;

@Profile("setup")
@Configuration
@ComponentScan("de.monticore.lang.monticar.setup")
public class ArmdilloSetupConfig {

  @Value("${armadillo.setup.url:https://github.com/conradsnicta/armadillo-code/archive/8.400.x.zip}")
  private URL armadilloURL;

  @Value("#{systemProperties['armadillo.setup.download'] ?: systemProperties['user.home']}/armadillo/armadillo.zip")
  private Path armadilloDownloadPath;

  @Value("#{systemProperties['armadillo.setup.extract'] ?: systemProperties['user.home']}/armadillo/")
  private Path armadilloExtractionPath;

  @Value("${armadillo.include:armadillo-code-8.400.x/include}")
  private Path armadilloInclude;

  @Bean
  public de.monticore.lang.monticar.setup.Configuration armadilloConfiguration() {
    return new BasicConfiguration(
        new DownloadAction(armadilloURL, armadilloDownloadPath),
        new ExtractionAction(armadilloDownloadPath, armadilloExtractionPath));
  }

  @Bean
  public Path armadilloInclude() {
    return armadilloExtractionPath.resolve(armadilloInclude).toAbsolutePath().normalize();
  }

}
