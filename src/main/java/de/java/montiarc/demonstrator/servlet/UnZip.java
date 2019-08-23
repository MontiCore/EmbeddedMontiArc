/* (c) https://github.com/MontiCore/monticore */
package de.java.montiarc.demonstrator.servlet;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

public class UnZip
{
    public String unZipIt(String zipFile, String outputFolder){

        byte[] buffer = new byte[4096];
        String projectFolderName = "";

        try{

            //create output directory is not exists
            File folder = new File(outputFolder);
            if(!folder.exists()){
                folder.mkdir();
            }

            //get the zip file content
            ZipInputStream zis = new ZipInputStream(new FileInputStream(zipFile));
            //get the zipped file list entry
            ZipEntry ze = zis.getNextEntry();

            while(ze!=null){

                if(ze.isDirectory()){
                    ze = zis.getNextEntry();
                }

                String fileName = ze.getName();
                File newFile = new File(outputFolder + File.separator + fileName);

                System.out.println("file unzip : "+ newFile.getAbsoluteFile());

                //create all non exists folders
                new File(newFile.getParent()).mkdirs();
                if(projectFolderName =="")
                    projectFolderName = new File(newFile.getParent()).getName();

                FileOutputStream fos = new FileOutputStream(newFile);

                int len;
                while ((len = zis.read(buffer)) > 0) {
                    fos.write(buffer, 0, len);
                }

                fos.close();
                ze = zis.getNextEntry();
            }

            zis.closeEntry();
            zis.close();

            System.out.println("Done");

        }catch(IOException ex){
            System.out.println("Error");
            ex.printStackTrace();
        }

        return projectFolderName;
    }

}
