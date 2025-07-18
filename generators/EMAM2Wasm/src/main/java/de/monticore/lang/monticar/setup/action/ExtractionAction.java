/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.setup.action;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import org.rauschig.jarchivelib.Archiver;
import org.rauschig.jarchivelib.ArchiverFactory;

public class ExtractionAction implements Action {

  private Path archive;
  private Path target;

  public ExtractionAction(Path archive, Path target) {
    this.archive = archive;
    this.target = target;
  }

  public Path getArchive() {
    return archive;
  }

  public Path getTarget() {
    return target;
  }


  @Override
  public void execute() throws ActionException {
    File file = archive.toFile();
    Archiver archiver = ArchiverFactory.createArchiver(file);
    try {
      archiver.extract(file, target.toFile());
    } catch (IOException e) {
      throw new ActionException("File extraction failed for " + archive, e);
    }
  }
}
