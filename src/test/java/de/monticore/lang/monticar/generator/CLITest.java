package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import org.junit.Test;

public class CLITest {
    @Test
    public void test1() {
        GeneratorCPP.main(new String[] {
                "test.basicConstantAssignment"
        , "src/test/resources",
                "target/cli"});
    }
}
