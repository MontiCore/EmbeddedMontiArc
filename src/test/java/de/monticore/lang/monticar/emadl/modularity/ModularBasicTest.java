package de.monticore.lang.monticar.emadl.modularity;

import de.se_rwth.commons.logging.Log;
import org.junit.Ignore;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class ModularBasicTest extends ModularTestSetup {
    String[] hashPaths = {"target/hashes"};


    @Test
    public void testEmtpyNetwork() throws IOException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/ModularCNN/emptyNetwork", "-r", "calculator.Connector", "-o", "target", "-b", "GLUON", "-c", "y", "-f", "n"};
        runGenerator(args, hashPaths,6,true);
    }

    @Test
    public void testModularNetworkSimpleMultiNet() throws IOException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/ModularCNN/modularNetworkSimpleMultiNet", "-r", "calculator.Connector", "-o", "target", "-b", "GLUON", "-c", "y", "-f", "n"};
        runGenerator(args, hashPaths,7,false);
    }

    @Test
    public void testModularNetworkComplex() throws IOException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/ModularCNN/modularNetworkComplex", "-r", "calculator.Connector", "-o", "target", "-b", "GLUON", "-c", "y", "-f", "n"};
        runGenerator(args, hashPaths,6,false);
    }

    @Test
    public void testModularNetworkSimple() throws IOException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/ModularCNN/modularNetworkSimple", "-r", "calculator.Connector", "-o", "target", "-b", "GLUON", "-c", "y", "-f", "n"};
        runGenerator(args, hashPaths,6,false);
    }

    @Test
    public void testSingleNetwork() throws IOException {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/ModularCNN/singleNetwork", "-r", "calculator.Connector", "-o", "target", "-b", "GLUON", "-c", "y", "-f", "n"};
        runGenerator(args, hashPaths,6,false);
    }
}
