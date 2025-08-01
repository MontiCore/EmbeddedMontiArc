/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math;

import de.se_rwth.commons.logging.Log;

/**
 */
public class LogConfig extends Log {
    static Log log;
    boolean disableOutput = false;

    public static void init() {
        log = new LogConfig();
        Log.setLog(log);
    }

    @Override
    protected void doInfo(String msg, Throwable t, String logName) {
        if (!disableOutput) {
            super.doInfo(msg, t, logName);
        }
    }

    @Override
    protected void doInfo(String msg, String logName) {
        if (!disableOutput) {
            super.doInfo(msg, logName);
        }
    }


    @Override
    protected void doDebug(String msg, Throwable t, String logName) {
        if (!disableOutput) {
            super.doDebug(msg, t, logName);
        }
    }

    @Override
    protected void doDebug(String msg, String logName) {
        if (!disableOutput) {
            super.doDebug(msg, logName);
        }
    }


    @Override
    protected void doTrace(String msg, Throwable t, String logName) {
        if (!disableOutput) {
            super.doTrace(msg, t, logName);
        }
    }

    @Override
    protected void doTrace(String msg, String logName) {
        if (!disableOutput) {
            super.doTrace(msg, logName);
        }
    }

}
