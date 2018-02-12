package de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper;

public class TestsEndWithTestResult {
    private String path;
    private boolean endsWithTest;

    public TestsEndWithTestResult(String path, boolean endsWithTest) {
        this.path = path;
        this.endsWithTest = endsWithTest;
    }

    public String getPath() {
        return path;
    }

    public boolean endsWithTest() {
        return endsWithTest;
    }
}
