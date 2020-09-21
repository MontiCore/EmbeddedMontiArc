package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic;

import org.junit.Test;

public class ConsistencyTest extends AbstractTest {


    public ConsistencyTest() {
        super("src/test/resources/test/embeddedmontiarcdynamic/consistency/");
    }

    @Test
    public void ConsistencyCheck_01_Valid() {
        checkValid("", "valid.SubComp");
        checkValid("", "valid.Test1");
    }

    @Test
    public void ConsistencyCheck_02_Warning() {

        checkValid("", "warnings.Test1", 1, "xCC005");
        checkValid("", "warnings.Test2", 1, "xCC004");
    }


    @Test
    public void ConsistencyCheck_03_Invalid() {
        checkValid("", "invalid.Test1", 1, "xCC007");
        checkValid("", "invalid.Test2", 2, "xCC002", "xCC004");
        checkValid("", "invalid.Test3", 1, "xCC001");

    }

    @Test
    public void ConsistencyCheck_04_Coverage() {
        checkValid("", "coverage.Test1", 14, "xCC007", "xCC004", "xCC002", "xCC005");
    }
}