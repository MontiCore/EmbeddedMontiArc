package de.monticore.montiarc.utilities.mavenpackage.tools;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

public class ZipFileCreator {



    public static boolean Zip(String path, Map<String,File> files){
        try{

            FileOutputStream fos = new FileOutputStream(path);
            ZipOutputStream zos = new ZipOutputStream(fos);

            for (Map.Entry<String,File> file: files.entrySet()) {
                //AddFileToZipFile(file.getCanonicalPath().replaceFirst(basePath, ""), file, zos);
                AddFileToZipFile(file.getKey(), file.getValue(),zos);
            }

            zos.close();
            fos.close();

        }catch (Exception ex){
            ///TODO: log entry
            ex.printStackTrace();

            return false;
        }
        return true;
    }

    protected static void AddFileToZipFile(String entryName, File file, ZipOutputStream zos) throws Exception{
        FileInputStream fis = new FileInputStream(file);
        ZipEntry zipEntry = new ZipEntry(entryName);
        zos.putNextEntry(zipEntry);
        byte[] buffer = new byte[1024];
        int length = 0;
        while((length = fis.read(buffer)) >= 0){
            zos.write(buffer, 0, length);
        }
        zos.closeEntry();
        fis.close();
    }

}
