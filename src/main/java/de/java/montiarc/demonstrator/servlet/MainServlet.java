package de.java.montiarc.demonstrator.servlet;

import java.io.*;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import org.eclipse.jetty.http.HttpStatus;

public class MainServlet extends HttpServlet {

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
        UnZip unZip = new UnZip();
        unZip.unZipIt("incomingData/source.zip","../emam2wasm/models");
        String modelName = unZip.unZipIt("incomingData/source.zip","../EmbeddedMontiArcStudio/model");

        // Compile to C, and run tests
        boolean res0 = true; //compileAndRunTest(modelName) == 0;

        //Compile sources, emam2wasm
        boolean res1 = compileEMAM(modelName) == 0;

        // if the compilation was successful then pack data and send back
        if(res0 && res1){

            //Read compiled files and pack them into archive
            ZipMultipleFiles zipOut = new ZipMultipleFiles();

            ByteArrayOutputStream zipStream = zipOut.zipItToStream(
                    "C:/Users/Administrator/code/WebServerForDemonstrator/outgoingData/mainController.wasm",
                    "C:/Users/Administrator/code/WebServerForDemonstrator/outgoingData/mainController.js");

            // Send response with encoded zipStream
            resp.addHeader("Access-Control-Allow-Origin", "*");
//            resp.setContentType("blob");
            resp.setStatus(HttpStatus.OK_200);
            resp.getOutputStream().write(zipStream.toByteArray());

            System.out.println("File sent back with size: " + zipStream.toByteArray().length);
        } else {

            resp.addHeader("Access-Control-Allow-Origin", "*");
            resp.setContentType("text/plain");
            resp.sendError(HttpStatus.INTERNAL_SERVER_ERROR_500,"Error during compilation! Check the model!");

        }

        clearWorkspace(new File("incomingData"));
        clearWorkspace(new File("../emam2wasm/models"));
        clearWorkspace(new File("../EmbeddedMontiArcStudio/model"));
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

    protected int compileEMAM(String modelName) throws IOException {

        Runtime rt = Runtime.getRuntime();
        String[] commands = {"C:\\Users\\Administrator\\code\\emam2wasm\\compile_notAll.bat",modelName + ".mainController"};
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

    protected int compileAndRunTest(String modelName) throws IOException {

        Runtime rt = Runtime.getRuntime();
        String[] commands = {"C:\\Users\\Administrator\\code\\EmbeddedMontiArcStudio\\compileRunExec.bat", modelName + ".mainController"};
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
