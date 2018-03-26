package de.monticore.lang.monticar.setup;

import de.monticore.lang.monticar.setup.action.Action;
import de.monticore.lang.monticar.setup.action.DownloadAction;
import de.monticore.lang.monticar.setup.action.ExtractionAction;
import java.net.URL;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class ArmadilloConfiguration implements Configuration {

  private URL armadillo;
  private Path filePath;
  private Path extractionPath;

  public ArmadilloConfiguration(URL armadillo, Path filePath, Path extractionPath) {
    this.armadillo = armadillo;
    this.filePath = filePath;
    this.extractionPath = extractionPath;
  }

  public List<Action> getActions() {
    List<Action> actions = new ArrayList<>();
    actions.add(new DownloadAction(armadillo, filePath));
    actions.add(new ExtractionAction(filePath, extractionPath));
    return actions;
  }
}
