/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities.tools;


import de.se_rwth.commons.logging.Log;

public class NoLog extends Log {

    private NoLog(){
    }

    public static void init(){
        Log.setLog(new NoLog());
    }

    @Override
    protected boolean doIsDebugEnabled(String logName) {
        return false;
    }

    @Override
    protected boolean doIsTraceEnabled(String logName) {
        return false;
    }

    @Override
    protected void doTrace(String msg, String logName) {
        //...
    }

    @Override
    protected void doDebug(String msg, String logName) {
        //...
    }

    @Override
    protected void doInfo(String msg, String logName) {
        //...
    }

}
