package de.monticore.lang.monticar.generator.testing;

import de.se_rwth.commons.logging.Log;

import java.io.IOException;

/**
 * @author Sascha Schneiders
 */
public class StreamTestExecution {

    public static void compileTests(String targetFullPath, String targetBasePath) throws IOException {
        Process p = Runtime.
                getRuntime().
                exec(new String[]{"compileCPPTests.bat", targetFullPath, targetBasePath + "/exec"});
        while (p.isAlive()) {
            if (Log.isInfoEnabled("")) {
                System.out.print((char) p.getInputStream().read());
            }
        }
    }

    public static void executeTests(String targetBasePath) throws IOException {
        Process p = Runtime.
                getRuntime().
                exec(new String[]{"executeStreamTest.bat", targetBasePath + "/exec"});
        while (p.isAlive()) {
            if (Log.isInfoEnabled("")) {
                System.out.print((char) p.getInputStream().read());
            }
        }
    }
}
