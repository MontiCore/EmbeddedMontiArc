package de.monticore.lang.monticar.utilities.utils;

import de.monticore.lang.monticar.utilities.models.FileLocation;

import java.io.*;
import java.util.List;
import java.util.Map;
import java.util.jar.JarEntry;
import java.util.jar.JarOutputStream;
import java.util.jar.Manifest;

public class JarCreator {

  public static File createArtifact(String jarFileName, Manifest manifest, Map<String, FileLocation> fileList) throws IOException {
    File jarFile = new File(jarFileName);
    jarFile.deleteOnExit();

    try (OutputStream outputStream = new FileOutputStream(jarFile);
         JarOutputStream jarOutputStream = new JarOutputStream(outputStream, manifest))
    {
      for (FileLocation file : fileList.values()) {
        copyFileToJarOutputStream(jarOutputStream, file.getJarLocation(), file.getSourceLocation());
        copyFileToJarOutputStream(jarOutputStream, file.getMetadataJarLocation(), file.getPropertiesLocation());
      }
    }

    return jarFile;
  }

  public static File createArtifact(String jarFileName, Manifest manifest, List<FileLocation> fileList) throws IOException {
    File jarFile = new File(jarFileName);
    jarFile.deleteOnExit();

    try (OutputStream outputStream = new FileOutputStream(jarFile);
         JarOutputStream jarOutputStream = new JarOutputStream(outputStream, manifest))
    {
      for (FileLocation file : fileList) {
        copyFileToJarOutputStream(jarOutputStream, file.getJarLocation(), file.getSourceLocation());
        if(file.getMetadataJarLocation() != null && file.getPropertiesLocation() != null){
          copyFileToJarOutputStream(jarOutputStream, file.getMetadataJarLocation(), file.getPropertiesLocation());
        }
      }
    }

    return jarFile;
  }

  private static void copyFileToJarOutputStream(JarOutputStream jarOutputStream, String locationInJar, String sourceFileLocation) throws IOException{
    int len;
    byte[] buffer = new byte[1024];

    JarEntry jarEntry = new JarEntry(locationInJar);
    jarOutputStream.putNextEntry(jarEntry);

    InputStream inputStream = new BufferedInputStream(new FileInputStream(sourceFileLocation));
    while ((len = inputStream.read(buffer, 0, buffer.length)) != -1) {
      jarOutputStream.write(buffer, 0, len);
    }
    inputStream.close();
    jarOutputStream.closeEntry();
  }

}
