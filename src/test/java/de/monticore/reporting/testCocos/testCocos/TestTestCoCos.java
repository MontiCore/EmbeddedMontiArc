package de.monticore.reporting.testCocos.testCocos;

import de.monticore.reporting.testCocos.TestCoCos;
import de.monticore.reporting.testCocos.helper.TestResult;
import de.monticore.reporting.testCocos.helper.TestResultPrinter;
import org.apache.commons.io.FileUtils;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class TestTestCoCos {

    @Ignore
    @Test
    public void testCoCosTester() throws IOException {
        int modelNumber = 126;
        int validNumber = 78;
        int notParsedNumber = 24;
        int notResolvedNumber = 11;
        String root = "src/test/";
        String zipDummy = "modelsDummy.zip";
        String comparisonResultsPath = "src/test/resources/comparisonResults/comparisonResults.json";
        String comparisonResultsMergedPath = "src/test/resources/comparisonResults/comparisonResultsMerged.json";
        String resultPath = "target/generatedTestResults/results.json";

        File   rootFile = new File(root);
        TestCoCos tcc = new TestCoCos();
        List<TestResult> testResults = tcc.testAllCocos(rootFile, zipDummy, "emam", "ema");
        int size = testResults.size();
        assertTrue("Models found: " + size + " ; Models expected: " + modelNumber, size == modelNumber);

        int valid = 0;
        int parse = 0;
        int resolve = 0;

        for(TestResult testResult: testResults) {
            if(testResult.isValid()) valid ++;
            if(testResult.getParsed() == -1) parse++;
            if(testResult.getResolve() == -1) resolve++;
        }

        assertTrue("Valid models: " + valid + " ; expected: " + validNumber, valid == validNumber);
        assertTrue("Models not correctly parsed: " + parse + " ; expected: " + notParsedNumber, parse == notParsedNumber);
        assertTrue("Models not correctly resolved: " + resolve + " ; expected: " + notResolvedNumber, resolve == notResolvedNumber);

        TestResultPrinter.printTestResults(testResults, resultPath, false);
        String generatedTestResults = FileUtils.readFileToString(new File(resultPath));
        TestResultPrinter.printTestResults(testResults, resultPath, true);
        String generatedTestResultsMerged = FileUtils.readFileToString(new File(resultPath));
        String comparisonResults = FileUtils.readFileToString(new File(comparisonResultsPath));
        String comparisonResultsMerged = FileUtils.readFileToString(new File(comparisonResultsMergedPath));

        assertTrue("Did not find comparison file", !comparisonResults.equals(""));
        assertTrue("Did not find comparison_merged file", !comparisonResultsMerged.equals(""));
        assertTrue("Did not find generated files", !generatedTestResults.equals(""));
        assertTrue("Did not find generated_merged files", !generatedTestResultsMerged.equals(""));
        boolean notMerged = removeLineEndings(comparisonResults).equals(removeLineEndings(generatedTestResults));
        boolean merged = removeLineEndings(comparisonResultsMerged).equals(removeLineEndings(generatedTestResultsMerged));
        assertTrue("Generated test results are not correct", notMerged);
        assertTrue("Generated merged test results are not correct", merged);
    }

    private String removeLineEndings(String str){
        return str.replace("\r\n", "").replace("\r","").replace("\n","");
    }
}
