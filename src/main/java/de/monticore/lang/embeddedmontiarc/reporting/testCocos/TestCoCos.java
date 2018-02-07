package de.monticore.lang.embeddedmontiarc.reporting.testCocos;

import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.GitHubHelper;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.SearchFiles;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResult;
import de.monticore.lang.embeddedmontiarc.reporting.testCocos.helper.TestResultPrinter;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.util.LinkedList;
import java.util.List;

public class TestCoCos {

    public static void main(String[] args){
        if(args.length == 0)
            Log.error("Program needs projects root as argument.");
        File root = new File(args[0]);
        if(!root.exists())
            Log.error("Cannot find dir: " + root.getAbsolutePath());

        Log.enableFailQuick(false);

        List<TestResult> testResults = new LinkedList<>();

        TestCoCos testCoCos = new TestCoCos();
        for(File project: root.listFiles()){
            if(project.isDirectory()){
                testResults.addAll(testCoCos.testAllCocos(project, "models1a6a7c6e450b6d996a79c701efdd4e69.zip", "emam", "m"));
            }
        }

        TestResultPrinter.printTestResults(testResults, "C:/Praktikum/report/data.json");
    }

    public List<TestResult> testAllCocos(File root, String zipName, String... fileType){
        List<TestResult> testResults = new LinkedList<>();

        for(File projectDir: root.listFiles()) {
            if (projectDir.isDirectory()) {
                List<File> files = SearchFiles.searchFiles(projectDir, fileType);
                Log.enableFailQuick(false);
                for(File file: files){
//                    GitHubHelper ghh = new GitHubHelper();
//                    String gitHubRoot = ghh.getGitHubRoot(projectDir);
                    CoCoTester ccT = new CoCoTester();
                    TestResult testResult = null;

                    testResult = ccT.testCoCos(file.getAbsolutePath());
//                    testResult.setPath(ghh.getHTMLTagOf(projectDir, file, gitHubRoot));
                    testResult.setPath(getVFSTag(projectDir, file, zipName));

                    testResults.add(testResult);
                }
            }
        }

        return testResults;
    }

    private String getVFSTag(File project, File file, String zipName) {
        String urlToZip;
        String zipName_;
        if(zipName == null)
            zipName_ = "models1a6a7c6e450b6d996a79c701efdd4e69.zip";
        else
            zipName_ = zipName;

        urlToZip = "https://raw.githubusercontent.com/EmbeddedMontiArc/reporting/gh-pages/" + zipName_;
        zipName_ = zipName_.substring(0, zipName_.lastIndexOf("."));
        String name = file.getAbsolutePath().substring(project.getAbsolutePath().length() - project.getName().length());
        String displayName = name;
        if(project.getAbsolutePath().contains("MontiSim"))
            displayName = "MontiSim/" + displayName;
        return "<a target='_blank' href='onlineIDE/api/load.html?mountPoint=EmbeddedMontiArc/reporting/" + zipName_ + "&url="
                + urlToZip + "&openFile=/" + name + "'>" + displayName + "</a>";
    }
}
