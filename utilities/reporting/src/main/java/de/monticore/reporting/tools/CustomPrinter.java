/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.tools;

import java.io.OutputStream;
import java.io.PrintStream;

public class CustomPrinter {

    private static CustomPrinter instance;
    private boolean isInit = false;
    private PrintStream oldOut;
    private PrintStream oldErr;
    private PrintStream dummyStream;

    public static void init() {
        getInstance().doInit();
    }

    private void doInit() {
        this.isInit = true;
        this.oldOut = System.out;
        this.oldErr = System.err;
        this.dummyStream = new PrintStream(new OutputStream(){
            public void write(int b) {
                // NO-OP
            }
        });
        System.setOut(dummyStream);
        System.setErr(dummyStream);
    }

    public static void end() {
        getInstance().doEnd();
    }

    private void doEnd() {
        if(!isInit) return;
        this.isInit = false;
        System.setOut(oldOut);
        System.setErr(oldErr);
    }

    public static void println(String msg) {
        getInstance().doPrintln(msg);
    }

    private void doPrintln(String msg) {
        if (!isInit) return;
        this.oldOut.println(msg);
    }

    public static void print(String msg) {
        getInstance().doPrint(msg);
    }

    private void doPrint(String msg) {
        if (!isInit) return;
//        System.setOut(oldOut);
        this.oldOut.print(msg);
//        System.setOut(dummyStream);
    }

    private static CustomPrinter getInstance(){
        if (instance == null) {
            instance = new CustomPrinter();
            return instance;
        }
        else return instance;
    }
}
