package de.monticore.reporting.testCocos.helper;

import java.io.OutputStream;
import java.io.PrintStream;

public class CustomPrinter {

    private static CustomPrinter instance;
    private boolean isInit = false;
    private PrintStream oldOut;
    private PrintStream oldErr;
    private PrintStream dummyStream;

    public static void init() {
        if (instance == null)
            instance = new CustomPrinter();
        instance.doInit();
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
        if (instance == null)
            instance = new CustomPrinter();
        instance.doEnd();
    }

    private void doEnd() {
        if(!isInit) return;
        this.isInit = false;
        System.setOut(oldOut);
        System.setErr(oldErr);
    }

    public static void println(String msg) {
        if (instance == null)
            instance = new CustomPrinter();
        instance.doPrintln(msg);
    }

    private void doPrintln(String msg) {
        if (!isInit) return;
        System.setOut(oldOut);
        System.out.println(msg);
        System.setOut(dummyStream);
    }

    public static void print(String msg) {
        if (instance == null)
            instance = new CustomPrinter();
        instance.doPrint(msg);
    }

    private void doPrint(String msg) {
        if (!isInit) return;
        System.setOut(oldOut);
        System.out.print(msg);
        System.setOut(dummyStream);
    }
}
