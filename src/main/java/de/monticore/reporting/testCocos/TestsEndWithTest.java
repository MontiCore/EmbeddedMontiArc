package de.monticore.reporting.testCocos;

import de.monticore.reporting.testCocos.helper.GitHubHelper;
import de.monticore.reporting.testCocos.helper.SearchFiles;
import de.monticore.reporting.testCocos.helper.TestResultPrinter;
import de.monticore.reporting.testCocos.helper.TestsEndWithTestResult;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.LinkedList;
import java.util.List;

public class TestsEndWithTest {

    public static void main(String[] args) {
        if (args.length == 0)
            Log.error("Please enter a path.");
        String path = args[0];

        File root = new File(path);
        if (!root.exists())
            Log.error("Path does not exist");

        TestsEndWithTest tewt = new TestsEndWithTest();
        List<TestsEndWithTestResult> testResults = tewt.testTestsEndWithTest(root);

        TestResultPrinter.printTestsEndWithTestResults(testResults, "C:/Praktikum/report/dataEWT.json");
    }

    public List<TestsEndWithTestResult> testTestsEndWithTest(File projectRoot) {
        List<TestsEndWithTestResult> testResults = new LinkedList<>();

        GitHubHelper ghh = new GitHubHelper();

        for (File project : projectRoot.listFiles()) {
            File testDirectory = new File(project.getAbsoluteFile() + "/src/test/java");
            if (testDirectory.exists()) {
                String gitHubRoot = ghh.getGitHubRoot(project);
                List<File> files = SearchFiles.searchFiles(testDirectory, "java");
                for (File file : files) {
                    // Check whether the file contains @Test
                    boolean containsTest = false;
                    try {
                        List<String> lines = Files.readAllLines(Paths.get(file.getAbsolutePath()));
                        for (String line : lines)
                            if (line.contains("@Test")) {
                                containsTest = true;
                                break;
                            }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }

                    if (containsTest) {
                        String htmlTag = ghh.getHTMLTagOf(project, file, gitHubRoot);
                        boolean passed = false;
                        String name = file.getName().substring(0, file.getName().length() - ".java".length());
                        if (name.endsWith("Test"))
                            passed = true;
                        else if (name.startsWith("Test") && !name.startsWith("Tests"))
                            passed = true;
                        else
                            passed = false;
                        testResults.add(new TestsEndWithTestResult(htmlTag, passed));
                    }
                }
            }
        }

        return testResults;
    }
}
