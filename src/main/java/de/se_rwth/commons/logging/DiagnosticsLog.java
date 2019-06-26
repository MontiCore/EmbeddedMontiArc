package de.se_rwth.commons.logging;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

public class DiagnosticsLog extends Log {
    static private DiagnosticsLog lastInstance = null;
    private boolean quickFail;
    private DateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss.SSS");
    private PrintWriter printWriter;
    static private boolean logToStdout = false;

    static public void setLogToStdout(boolean logToStdout) {
        DiagnosticsLog.logToStdout = logToStdout;
        Log.info("LogToStdout status: " + logToStdout, "default");
    }

    public static void clearAndUse(){
        reuse();
        Log.getFindings().clear();
    }

    public static void init() {
        DiagnosticsLog l = new DiagnosticsLog();
        l.quickFail = false;
        l.isDEBUG = false;
        l.isTRACE = false;
        try {
            Path path = Files.createTempFile("DiagnosticsLog", ".txt");
            l.printWriter = new PrintWriter(new FileWriter(path.toFile(), true));
        } catch (IOException e) {
            //IGNORE
        }
        lastInstance = l;
        setLog(l);
        Log.info("Initialized Log", "default");
    }

    public static void reuse(){
        if(lastInstance == null){
            init();
        }
        Log.setLog(lastInstance);
    }

    protected void doPrint(String msg) {
        if(printWriter != null) {
            printWriter.println(dateFormat.format(new Date()) + " " + msg);
            printWriter.flush();
        }
        if(logToStdout){
            System.out.println(dateFormat.format(new Date()) + " " + msg);
        }
    }

    @Override
    protected boolean doIsFailQuickEnabled() {
        return quickFail;
    }
}
