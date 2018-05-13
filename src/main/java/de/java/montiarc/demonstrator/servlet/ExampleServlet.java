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
    protected void doOptions(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
        super.doOptions(req, resp);
        resp.addHeader("Access-Control-Allow-Origin","*");
        resp.addHeader("Access-Control-Allow-Headers", "Content-Type");
    }

    @Override
    protected void doPost(HttpServletRequest request, HttpServletResponse resp)
            throws ServletException, IOException {

        // Clean data from previous session
        clearWorkspace(new File("outgoingData"));

        // Write down file to the disk
        new File("incomingData").mkdirs();

        File saveFile = new File("incomingData/source.zip");
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
        unZip.unZipIt("incomingData/source.zip","incomingDataExtracted/simulator1");

        //Compile sources

        compileEMAM();

        //Read compiled files and pack them into archive

        ZipMultipleFiles zipOut = new ZipMultipleFiles();

        ByteArrayOutputStream zipStream = zipOut.zipItToStream(
                "C:/Users/Administrator/code/WebServerForDemonstrator/outgoingData/mainController.wasm",
                "C:/Users/Administrator/code/WebServerForDemonstrator/outgoingData/mainController.js");

//        Read data from file
//        zipOut.zipItToFile(
//                "/home/streug/code/demonstrator/mainController.wasm",
//                "/home/streug/code/demonstrator/mainController.js");
//        Path path = Paths.get("multiCompressed.zip");
//        byte[] fileBytes = Files.readAllBytes(path);

        // Send response with encoded zipStream
        resp.addHeader("Access-Control-Allow-Origin", "*");
        resp.setContentType("blob");
        resp.setStatus(HttpStatus.OK_200);
        resp.getOutputStream().write(zipStream.toByteArray());

        System.out.println("File sent back with size: " + zipStream.toByteArray().length);

        clearWorkspace(new File("incomingData"));
        clearWorkspace(new File("../emam2wasm/models"));
    }

    protected String generateName(){

        String name = "file" + Integer.toString(nameNumber);
        nameNumber++;

        return name;
    }

    protected Integer nameNumber = 0;

    protected void clearWorkspace(File dir) {

        for(File file: dir.listFiles()) {
            if (file.isDirectory()) this.clearWorkspace(file);
            if (file.isDirectory()) System.out.print("Folder ");
                else System.out.print("File ");
            System.out.println(dir.getName()+"/"+file.getName()+" will be deleted!");
            file.delete();
        }

    }

    protected int compileEMAM() throws IOException {

        Runtime rt = Runtime.getRuntime();
        String[] commands = {"C:\\Users\\Administrator\\code\\emam2wasm\\compile_notAll.bat"};
        Process proc = rt.exec(commands);

        BufferedReader stdInput = new BufferedReader(new
                InputStreamReader(proc.getInputStream()));

        BufferedReader stdError = new BufferedReader(new
                InputStreamReader(proc.getErrorStream()));

        // read the output from the command
        System.out.println("Here is the standard output of the command:\n");
        String s = null;
        while ((s = stdInput.readLine()) != null) {
            System.out.println(s);
        }

        // read any errors from the attempted command
        System.out.println("Here is the standard error of the command (if any):\n");
        while ((s = stdError.readLine()) != null) {
            System.out.println(s);
        }

        return proc.exitValue();
    }
}
