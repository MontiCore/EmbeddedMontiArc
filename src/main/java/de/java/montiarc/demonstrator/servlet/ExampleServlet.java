package de.java.montiarc.demonstrator.servlet;

import java.io.*;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import java.time.Instant;
import java.time.Duration;
import java.util.Base64;

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

        // Compiling the wasm file
//        // Measure time
//        Instant start = Instant.now();
//
//        Runtime rt = Runtime.getRuntime();
//        String[] commands = {"C:\\Users\\Administrator\\code\\emam2wasm\\compile_notAll.bat"};
//        Process proc = rt.exec(commands);
//
//        BufferedReader stdInput = new BufferedReader(new
//                InputStreamReader(proc.getInputStream()));
//
//        BufferedReader stdError = new BufferedReader(new
//                InputStreamReader(proc.getErrorStream()));
//
//        // read the output from the command
//        System.out.println("Here is the standard output of the command:\n");
//        String s = null;
//        while ((s = stdInput.readLine()) != null) {
//            System.out.println(s);
//        }
//
//        // read any errors from the attempted command
//        System.out.println("Here is the standard error of the command (if any):\n");
//        while ((s = stdError.readLine()) != null) {
//            System.out.println(s);
//        }
//
//        Instant end = Instant.now();
//        //System.out.println(Duration.between(start, end));

        // TODO: read compiled files and pack them into archive

        ZipMultipleFiles zipOut = new ZipMultipleFiles();

        // TODO: modify paths to the files
        ByteArrayOutputStream zipStream = zipOut.zipIt(
                "/home/streug/code/demonstrator/mainController.wasm",
                "/home/streug/code/demonstrator/mainController.js");

        Base64.getEncoder().encodeToString(zipStream.toByteArray());

        // Send response with encoded zipStream in base64
        resp.setStatus(HttpStatus.OK_200);
        resp.getWriter().println(Base64.getEncoder().encodeToString(zipStream.toByteArray()));
    }

    protected String generateName(){

        String name = "file" + Integer.toString(nameNumber);
        nameNumber++;

        return name;
    }

    protected Integer nameNumber = 0;
}
