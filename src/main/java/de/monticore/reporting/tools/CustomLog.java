/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.tools;

public class CustomLog extends de.se_rwth.commons.logging.Log {
    @Override
    protected void doTrace(String msg, String logName) {
        if (this.doIsTraceEnabled(logName)) {
            this.doPrint("[TRACE] " + logName + " " + msg);
        }
    }

    public static void initThis(){
        setLog(new CustomLog());
    }
}
