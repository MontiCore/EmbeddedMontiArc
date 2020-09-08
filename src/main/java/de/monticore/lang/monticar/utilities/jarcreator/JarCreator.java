package de.monticore.lang.monticar.utilities.jarcreator;

import java.io.*;
import java.util.List;
import java.util.jar.JarEntry;
import java.util.jar.JarOutputStream;
import java.util.jar.Manifest;

public class JarCreator {

  public static void create(String jarFileName, Manifest manifest, List<FileLocation> fileList) {
    File jarFile = new File(jarFileName);

    try (OutputStream outputStream = new FileOutputStream(jarFile);
         JarOutputStream jarOutputStream = new JarOutputStream(outputStream, manifest))
    {
      int len;
      byte[] buffer = new byte[1024];
      for (FileLocation file : fileList) {
        JarEntry jarEntry = new JarEntry(file.getJarLocation());
        jarOutputStream.putNextEntry(jarEntry);

        InputStream inputStream = new BufferedInputStream(new FileInputStream(file.getSourceLocation()));
        while ((len = inputStream.read(buffer, 0, buffer.length)) != -1) {
          jarOutputStream.write(buffer, 0, len);
        }
        inputStream.close();
        jarOutputStream.closeEntry();
      }
    }
    catch (IOException e) {
      e.printStackTrace();
    }

  }

}
