package de.monticore.lang.gdl;

import java.io.File;

import org.junit.Test;

public class MatchTests implements GDLTest.TestImpl {

    @Test
    public void testMatches() throws Exception {
        File testDir = new File("src/test/resources/test-matches/");
        GDLTest gdlTest = new GDLTest(this);
        for (File testFile : testDir.listFiles()) {
            gdlTest.testMatch(testFile);
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
