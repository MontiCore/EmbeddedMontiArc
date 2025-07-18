/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.cocoReport.helper;

public class CheckTestResult {
    private String path;
    private boolean endsWithTest;

    public CheckTestResult(String path, boolean endsWithTest) {
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
