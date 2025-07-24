/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.util;

import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;

public class FileDownloader {

  public static Path download(URL url, Path target) throws IOException {
    Files.createDirectories(target.getParent());
    try (InputStream in = url.openStream()) {
      Files.copy(in, target, StandardCopyOption.REPLACE_EXISTING);
    }
    return target;
  }

}
