/* (c) https://github.com/MontiCore/monticore */
package de.java.montiarc.demonstrator.servlet;

import java.io.*;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.List;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

public class ZipMultipleFiles {

    public ByteArrayOutputStream zipItToStream(String... files) throws IOException {

        List<String> srcFiles = Arrays.asList(files);
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        ZipOutputStream zipOut = new ZipOutputStream(baos);

        for (String srcFile : srcFiles) {

            File fileToZip = new File(srcFile);
            ByteArrayInputStream fis = new ByteArrayInputStream(Files.readAllBytes(fileToZip.toPath()));
            ZipEntry zipEntry = new ZipEntry(fileToZip.getName());

            zipOut.putNextEntry(zipEntry);

            byte[] bytes = new byte[1024];
            int length;
            while((length = fis.read(bytes)) >= 0) {
                zipOut.write(bytes, 0, length);
            }
            fis.close();

            System.out.println(srcFile + " has been packed");
        }

        zipOut.close();
        baos.close();

        return baos;
    }

    public void zipItToFile(String... files) throws IOException {

        List<String> srcFiles = Arrays.asList(files);
        FileOutputStream fos = new FileOutputStream("multiCompressed.zip");
        ZipOutputStream zipOut = new ZipOutputStream(fos);

        for (String srcFile : srcFiles) {
            File fileToZip = new File(srcFile);
            FileInputStream fis = new FileInputStream(fileToZip);
            ZipEntry zipEntry = new ZipEntry(fileToZip.getName());
            zipOut.putNextEntry(zipEntry);

            byte[] bytes = new byte[1024];
            int length;
            while((length = fis.read(bytes)) >= 0) {
                zipOut.write(bytes, 0, length);
            }
            fis.close();
        }
        zipOut.close();
        fos.close();
    }
}
