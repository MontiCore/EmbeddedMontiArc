/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.testReport;

import de.monticore.reporting.tools.GitLabHelper;
import de.monticore.reporting.tools.SearchFiles;
import de.monticore.reporting.cocoReport.helper.CheckTestResult;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.LinkedList;
import java.util.List;

public class CheckTests {

    public List<CheckTestResult> testTestsEndWithTest(File projectRoot) {
        List<CheckTestResult> testResults = new LinkedList<>();

        GitLabHelper ghh = new GitLabHelper();

        for (File project : projectRoot.listFiles()) {
            File testDirectory = new File(project.getAbsoluteFile() + "/src/test/java");
            if (testDirectory.exists()) {
                String gitHubRoot = ghh.getGitLabRoot(project);
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
                        testResults.add(new CheckTestResult(htmlTag, passed));
                    }
                }
            }
        }

        return testResults;
    }
}
