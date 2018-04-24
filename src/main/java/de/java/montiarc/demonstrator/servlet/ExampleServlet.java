package de.java.montiarc.demonstrator.servlet;

import java.io.*;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import org.eclipse.jetty.http.HttpStatus;

public class ExampleServlet extends HttpServlet {

    @Override
    protected void doGet(HttpServletRequest req, HttpServletResponse resp)
            throws ServletException, IOException {

        resp.setStatus(HttpStatus.OK_200);
        resp.getWriter().println("To send a file please use POST request.");
    }

    @Override
    protected void doPost(HttpServletRequest request, HttpServletResponse resp)
            throws ServletException, IOException {


        // Write down file to the disk
        String fileName = this.generateName();

        new File("incomingData").mkdirs();

        File saveFile = new File("incomingData/"+fileName+".zip");
        InputStream inputStream = request.getInputStream();
        FileOutputStream outputStream = new FileOutputStream(saveFile);

        byte[] buffer = new byte[32768];
        System.out.println("Receiving data...");

        int len = inputStream.read(buffer);
        System.out.println("Length - " + len);

        while (len != -1) {
            outputStream.write(buffer, 0, len);
            len = inputStream.read(buffer);
        }

        System.out.println("Data received.");
        outputStream.close();
        inputStream.close();


        //Unzip works here, just read file from current dir
        // TODO: fix the problem with extracting an archive which contents a folder
        UnZip unZip = new UnZip();
        unZip.unZipIt("incomingData/"+fileName+".zip","incomingDataExtracted/"+fileName);

        // Send response
        resp.setStatus(HttpStatus.OK_200);
        resp.getWriter().println("Data has been received.");
    }

    protected String generateName(){

        String name = "file" + Integer.toString(nameNumber);
        nameNumber++;

        return name;
    }

    protected Integer nameNumber = 0;
}
