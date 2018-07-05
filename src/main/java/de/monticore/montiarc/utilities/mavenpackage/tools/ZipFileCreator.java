package de.monticore.montiarc.utilities.mavenpackage.tools;

import java.io.*;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
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
        byte[] buffer = new byte[2048];
        int length = 0;
        while((length = fis.read(buffer)) >= 0){
            zos.write(buffer, 0, length);
        }
        zos.closeEntry();
        fis.close();
    }


    public static void ExtractTo(String pathZIP, String pathOUT) throws IOException {
        InputStream theFile = new FileInputStream(pathZIP);
        ZipInputStream stream = new ZipInputStream(theFile);
        byte[] buffer = new byte[2048];
        if(pathOUT.endsWith("/")){
            pathOUT = pathOUT.substring(0, pathOUT.length()-1);
        }
        try
        {
            ZipEntry entry;
            while((entry = stream.getNextEntry())!=null)
            {



                Path path = Paths.get(pathOUT, entry.getName());

                FileOutputStream output = null;
                try
                {


                    File f = path.toFile(); //new File(outpath);
                    if(entry.isDirectory()){
                        f.mkdirs();
                    }else{
                        f.getParentFile().mkdirs();
                        f.createNewFile();
                    }

                    output = new FileOutputStream(path.toFile().getCanonicalPath());
                    int len = 0;
                    while ((len = stream.read(buffer)) > 0)
                    {
                        output.write(buffer, 0, len);
                    }
                }
                finally
                {
                    if(output!=null) output.close();
                }
            }
        }
        finally
        {
            // we must always close the zip file.
            stream.close();
        }

    }
}

