import java.io.File;
import java.io.IOException;

import org.junit.Test;

import de.monticore.lang.gdl.GDLTest;

public class MatchTests implements GDLTest.TestImpl {

    private final GDLTest TESTER = new GDLTest(this);

    @Test
    public void testSnimmtTyped() {
        File testDir = new File("MatchTests/6nimmtTyped");
        for (File testFile : testDir.listFiles()) {
            doTestForFile(testFile);
        }
    }

    @Test
    public void testSnimmt() {
        File testDir = new File("MatchTests/6nimmt");
        for (File testFile : testDir.listFiles()) {
            doTestForFile(testFile);
        }
    }

    public void doTestForFile(File testFile) {
        try {
            TESTER.testMatch(testFile);
        } catch (IOException e) {
            System.err.println("An unexpected error occured");
            org.junit.Assert.assertTrue(false);
        }
    }

    @Override
    public void assertEquals(String message, Object expected, Object actual) {
        org.junit.Assert.assertEquals(message, expected, actual);
    }

    @Override
    public void assertNotNull(Object object) {
        org.junit.Assert.assertNotNull(object);
    }

    @Override
    public void assertTrue(String message, boolean condition) {
        org.junit.Assert.assertTrue(message, condition);
    }

}
