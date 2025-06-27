/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities.tools;

import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.Optional;

public class LogToFile extends Log {



    protected Optional<Path> logFilePath;


    private LogToFile(){
        logFilePath = Optional.empty();
    }

    public static LogToFile initFile(){
        LogToFile ltf = new LogToFile();
        //Log.setLog(new LogToFile());
        Log.setLog(ltf);
        return ltf;
    }

    public static void resetTo(LogToFile ltf){
        Log.setLog(ltf);
    }

    public void setLogFile(String path){
        if(!logFilePath.isPresent()) {
            logFilePath = Optional.of(Paths.get(path));
        }
    }

    public void clear()  {
        if(logFilePath.isPresent()){
            try {
                Files.write(logFilePath.get(), "".getBytes());
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    protected void write(String line){
        if(logFilePath.isPresent()){
            try {
                Files.write(logFilePath.get(), line.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    protected void doDebug(String msg, String logName) {
        this.write("[debug] "+logName+" : "+msg+System.lineSeparator());
    }

    @Override
    protected void doInfo(String msg, String logName) {
        this.write("[info] "+logName+" : "+msg+System.lineSeparator());
    }

    @Override
    protected void doWarn(String msg) {
        Finding warn = Finding.warning(msg);
        this.addFinding(warn);
        this.write("[warn] "+warn.toString()+System.lineSeparator());
    }

    @Override
    protected void doError(String msg, Throwable t) {
        Finding error = Finding.error(msg);
        this.addFinding(error);
        //System.err.println("[ERROR] " + error.toString());
        this.write("[error] "+error.toString()+System.lineSeparator());
        t.printStackTrace(System.err);
        this.terminateIfErrors();
    }

    @Override
    protected void doError(String msg, SourcePosition pos) {
        Finding error = Finding.error(msg, pos);
        this.addFinding(error);
        //System.err.println("[ERROR] " + error.toString());
        this.write("[error] "+error.toString()+System.lineSeparator());
        this.terminateIfErrors();
    }

    @Override
    protected void doError(String msg) {
        Finding error = Finding.error(msg);
        this.addFinding(error);
        //System.err.println("[ERROR] " + error.toString());
        this.write("[error] "+error.toString()+System.lineSeparator());
        this.terminateIfErrors();
    }

    @Override
    protected void doTrace(String msg, String logName) {
        this.write("[trace] "+logName+" : "+msg+System.lineSeparator());
    }



}
