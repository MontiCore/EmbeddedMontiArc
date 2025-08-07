/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.setup.action;

import de.monticore.lang.monticar.util.FileDownloader;
import java.io.IOException;
import java.net.URL;
import java.nio.file.Path;

public class DownloadAction implements Action {

  private URL url;
  private Path fileDownloadPath;

  public DownloadAction(URL url, Path fileDownloadPath) {
    this.url = url;
    this.fileDownloadPath = fileDownloadPath;
  }

  public URL getUrl() {
    return url;
  }

  public Path getFileDownloadPath() {
    return fileDownloadPath;
  }

  @Override
  public void execute() throws ActionException {
    try {
      FileDownloader.download(url, fileDownloadPath);
    } catch (IOException e) {
      throw new ActionException(
          "Download of " + url + " to " + fileDownloadPath + " failed", e);
    }
  }
}
