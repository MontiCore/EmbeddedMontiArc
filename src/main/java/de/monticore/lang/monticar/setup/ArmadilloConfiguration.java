package de.monticore.lang.monticar.setup;

import de.monticore.lang.monticar.setup.action.Action;
import de.monticore.lang.monticar.setup.action.DownloadAction;
import de.monticore.lang.monticar.setup.action.ExtractionAction;
import java.net.URL;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

@Component
public class ArmadilloConfiguration implements Configuration {

  @Value("https://github.com/conradsnicta/armadillo-code/archive/8.400.x.zip")
  private URL armadillo;

  @Value("#{systemProperties['user.home']}/armadillo/armadillo.zip")
  private Path filePath;

  @Value("#{systemProperties['user.home']}/armadillo/")
  private Path extractionPath;

  public List<Action> getActions() {
    List<Action> actions = new ArrayList<>();
    actions.add(new DownloadAction(armadillo, filePath));
    actions.add(new ExtractionAction(filePath, extractionPath));
    return actions;
  }
}
