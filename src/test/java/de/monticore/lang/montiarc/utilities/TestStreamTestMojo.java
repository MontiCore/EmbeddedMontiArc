package de.monticore.lang.montiarc.utilities;

import de.monitcore.lang.montiarc.utilities.GeneratorEnum;
import de.monitcore.lang.montiarc.utilities.StreamTestMojo;
import de.monitcore.lang.montiarc.utilities.tools.SearchFiles;
import de.monticore.lang.montiarc.utilities.tools.ChecksumChecker;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;

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

@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class TestStreamTestMojo {

    protected static StreamTestMojo getNewStreamTestMojo(String path, String pathOut){

        StreamTestMojo stm = new StreamTestMojo();

        stm.setPathMain(path+"/main");
        stm.setPathTest(path+"/test");

        stm.setPathTmpOut(pathOut);

        stm.setWrapperTestExtension("_TestWrapper");

        //stm.setGpp("g++");
        //stm.setCppInludePaths(new String[]{});
        //stm.setUsemingw(false);
        //
        //stm.addCppIncludePaths("/usr/local/Cellar/armadillo/8.500.1/include");

        stm.setGenerator(GeneratorEnum.MinGW);

        //use this in test to see all output
        stm.setShowBuildAndRunOutput(true);

        return stm;
    }

    @Test
    public void Test_01_BuildFiles(){



        String path = "./target/tmp/Test_BuildFiles";
        Map<String,String> checksums = new HashMap<>();
        checksums.put(path+"/1/cpp/build.bat", "265A13D9EF578E8EEC87DB4AD083FF6731A6DEE37B6587BAB679FB17A667C927");
        checksums.put(path+"/1/cpp/build.sh", "0387937222E3D918568596DAD2B86330ACAB610F00DFF94D73207CE3A615081C");
        Log.info("BuildFiles 1 : NONE", "");
        this.InnerTest_01_BuildFiles(GeneratorEnum.NONE, path+"/1", checksums);

        checksums.clear();
        checksums.put(path+"/2/cpp/build.bat", "9C7C2B9C55502BB4429269A54E346AAD6020E3E50C1C33A3318E56F02A4D6FD8");
        checksums.put(path+"/2/cpp/build.sh", "0387937222E3D918568596DAD2B86330ACAB610F00DFF94D73207CE3A615081C");
        Log.info("BuildFiles 2 : VS2017", "");
        this.InnerTest_01_BuildFiles(GeneratorEnum.VS2017, path+"/2", checksums);

        checksums.clear();
        checksums.put(path+"/3/cpp/build.bat", "9C7C2B9C55502BB4429269A54E346AAD6020E3E50C1C33A3318E56F02A4D6FD8");
        checksums.put(path+"/3/cpp/build.sh", "0387937222E3D918568596DAD2B86330ACAB610F00DFF94D73207CE3A615081C");
        Log.info("BuildFiles 3 : VisualStudio2017", "");
        this.InnerTest_01_BuildFiles(GeneratorEnum.VisualStudio2017, path+"/3", checksums);

        checksums.clear();
        checksums.put(path+"/4/cpp/build.bat", "41D6D2B269B0B3A5E4D72F296A87604A52D9957680CF2A0074DEB39F9B2D0744");
        checksums.put(path+"/4/cpp/build.sh", "0387937222E3D918568596DAD2B86330ACAB610F00DFF94D73207CE3A615081C");
        Log.info("BuildFiles 4 : MinGW", "");
        this.InnerTest_01_BuildFiles(GeneratorEnum.MinGW, path+"/4", checksums);
    }

    private void InnerTest_01_BuildFiles(GeneratorEnum ge, String path, Map<String,String> checksums){

        StreamTestMojo stm = getNewStreamTestMojo("", path);
        stm.setGenerator(ge);
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
    public void Test_02_setupTmpFolder(){
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
    public void Test_03_execute_ParserTests(){
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
    public void Test_04_execution_valid() {
        //valid
        ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/Test_execution/valid");
    }

    @Test
    public void Test_05_execution_invalid() {

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

    @Test
    public void Test_06_execution_with_struct(){
        ValidInner("./src/test/resources/emam/struct", "./target/tmp/struct");
    }

    @Test
    public void Test_10_execution_many_combined() {
/*
        StreamTestMojo stm = getNewStreamTestMojo("./src/test/resources/emam/execution/many", "./target/tmp/Test_execution/manybuild");
        stm.setCombinebuilds(true);
        stm.setGenerator(GeneratorEnum.MinGW);
        try {
            stm.execute();
        } catch (MojoExecutionException e) {
            e.printStackTrace();
            fail(e.getMessage());
        } catch (MojoFailureException e) {
            e.printStackTrace();
            fail(e.getMessage());
        }
        */
    }

    /*@Test
    public void Test_11_execution_many() {
        //ValidInner("./src/test/resources/emam/execution/many", "./target/tmp/Test_execution/many");
    }*/

    protected void ValidInner(String path, String tmp){
        //valid
        StreamTestMojo stm = getNewStreamTestMojo(path, tmp);

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



}