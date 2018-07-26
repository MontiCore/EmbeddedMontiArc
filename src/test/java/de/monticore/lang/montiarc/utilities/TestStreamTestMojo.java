package de.monticore.lang.montiarc.utilities;

import de.monitcore.lang.montiarc.utilities.StreamTestMojo;
import de.monitcore.lang.montiarc.utilities.tools.SearchFiles;
import de.monticore.lang.montiarc.utilities.tools.ChecksumChecker;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.junit.Test;

import javax.xml.bind.DatatypeConverter;
import java.io.*;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static junit.framework.TestCase.assertTrue;
import static junit.framework.TestCase.fail;

public class TestStreamTestMojo {

    protected static StreamTestMojo getNewStreamTestMojo(String path, String pathOut){

        StreamTestMojo stm = new StreamTestMojo();

        stm.setPathMain(path+"/main");
        stm.setPathTest(path+"/test");

        stm.setPathTmpOut(pathOut);

        stm.setWrapperTestExtension("_TestWrapper");

        stm.setGpp("g++");
        stm.setCppInludePaths(new String[]{});
        //
        //stm.addCppIncludePaths("/usr/local/Cellar/armadillo/8.500.1/include");

        return stm;
    }



    @Test
    public void Test_execute_ParserTests(){
        String path = "./src/test/resources/emam/execute_ParserTests";
        String pathValid = path+"/valid";
        String pathInvalid = path+"/invalid";;

        //test for valid emam and stream files
        StreamTestMojo stm = getNewStreamTestMojo("", "./target/tmp/Test_BuildFiles");
        stm.setPathMain(path);
        stm.setPathTest(path);
        stm.init();

        Map<String, File> mainFiles = null;
        List<String> errorFiles = null;
        try {
            mainFiles = SearchFiles.searchFilesMap(pathValid, "emam", "stream");
            errorFiles = stm.execute_ParserTests(mainFiles);
            assertTrue("execute ParserTests for valid not successfull.", errorFiles.isEmpty());

            mainFiles = SearchFiles.searchFilesMap(pathInvalid, "emam", "stream");
            errorFiles = stm.execute_ParserTests(mainFiles);
            assertTrue("execute ParserTests for invalid not successfull.", !errorFiles.isEmpty());
        } catch (MojoExecutionException e) {
            fail(e.getMessage());
        }


        try {
            mainFiles = SearchFiles.searchFilesMap(path, "none");
            errorFiles = stm.execute_ParserTests(mainFiles);
            fail("Invalid file ending is parsable!");
        } catch (MojoExecutionException e) {
            // alles gut
        }

    }

    @Test
    public void Test_setupTmpFolder(){
        String path = "./src/test/resources/emam/oneTest";
        String pathTMP = "./target/tmp/Test_setupTmpFolder";
        //test for valid emam and stream files
        StreamTestMojo stm = getNewStreamTestMojo(path, pathTMP);
        stm.init();

        try {
            stm.setupTmpFolder();
        } catch (MojoExecutionException e) {
            e.printStackTrace();
            fail("Can't create test folder");
        }

        Map<String, File> mainFiles = SearchFiles.searchFilesMap(path+"/main", "emam", "stream");
        mainFiles.putAll(SearchFiles.searchFilesMap(path+"/test", "emam", "stream"));

        Map<String, File> tmpFiles = SearchFiles.searchFilesMap(pathTMP+"/emam", "emam", "stream");

        for (String entry:mainFiles.keySet()){
            assertTrue("File not in out tmp folder ("+entry+")", tmpFiles.containsKey(entry));
        }

    }

    @Test
    public void Test_BuildFiles(){
        String path = "./target/tmp/Test_BuildFiles";
        Map<String,String> checksums = new HashMap<>();
        checksums.put(path+"/1/cpp/build.bat", "4387BEC40756E17284DC93E7E299CE236BA0DFC01FF8F5FB34B5234DF962B06C");
        checksums.put(path+"/1/cpp/build.sh", "2D3F9B8908B3F69BF9DB635993FCBF5572C813789A2D836FCAC5A09E8CB8FC7C");


        StreamTestMojo stm = getNewStreamTestMojo("", path+"/1");
        stm.setCppInludePaths(new String[]{});
        stm.addCppIncludePaths("/a");
        stm.addCppIncludePaths("/b");
        stm.addCppIncludePaths("/c");

        try {
            stm.createBuildFiles();
        } catch (MojoExecutionException e) {
            e.printStackTrace();
            fail(e.getMessage());
        }

        try {
            ChecksumChecker.checkFilesWithAssert(checksums);
        } catch (IOException e) {
            e.printStackTrace();
            fail(e.getMessage());
        }


        checksums.clear();
        checksums.put(path+"/2/cpp/build.bat", "D715693FC74CA1152AE03B743CF9AC275EC63BF837AC3718507DE1D91F5CDD7F");
        checksums.put(path+"/2/cpp/build.sh", "B2C6C5FE18F3FC53AA9F685665DBF9BF713C3363C16CA850A9DB974CF2CA3516");


        stm = getNewStreamTestMojo("", path+"/2");
        stm.setCppInludePaths(new String[]{});

        try {
            stm.createBuildFiles();
        } catch (MojoExecutionException e) {
            e.printStackTrace();
            fail(e.getMessage());
        }

        try {
            ChecksumChecker.checkFilesWithAssert(checksums);
        } catch (IOException e) {
            e.printStackTrace();
            fail(e.getMessage());
        }

    }

    @Test
    public void Test_execution_valid() {

        //valid
        StreamTestMojo stm = getNewStreamTestMojo("./src/test/resources/emam/execution/valid", "./target/tmp/Test_execution/valid");

        try {
            stm.execute();
        } catch (MojoExecutionException e) {
            e.printStackTrace();
            fail(e.getMessage());
        } catch (MojoFailureException e) {
            e.printStackTrace();
            fail(e.getMessage());
        }
    }

    @Test
    public void Test_execution_invalid() {

        //valid
        StreamTestMojo stm = getNewStreamTestMojo("./src/test/resources/emam/execution/invalid", "./target/tmp/Test_execution/invalid");

        try {
            stm.execute();
            fail("A MojoFailureException should be thrown ");
        } catch (MojoExecutionException e) {
            e.printStackTrace();
            fail(e.getMessage());
        } catch (MojoFailureException e) {
            //should visit this case
        }
    }


}