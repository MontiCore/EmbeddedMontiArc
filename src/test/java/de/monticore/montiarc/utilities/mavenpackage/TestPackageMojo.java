package de.monticore.montiarc.utilities.mavenpackage;

import de.monticore.montiarc.utilities.mavenpackage.tools.SearchFiles;
import de.monticore.montiarc.utilities.mavenpackage.tools.ZipFileCreator;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.codehaus.plexus.util.FileUtils;
import org.hamcrest.Condition;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class TestPackageMojo {

    @Test
    public void test_filesToPack(){
        PackageMojo pm = new PackageMojo();
        Set<String> testSet = new HashSet<String>();
        testSet.add("ema");
        testSet.add("emam");
        testSet.add("stream");
        testSet.add("struct");
        testSet.add("enum");
        pm.setFilesToPack(String.join(",", testSet));

        String[] result = pm.filesToPackAsArray();

        assertTrue("Length mismatch ", 5 == result.length);

        for (String test:result) {
            assertTrue("Can't find "+test+" in testset.", testSet.contains(test));
        }

        for(String test: testSet){
            boolean found = false;
            for(int i = 0; ((!found) && (i < result.length)); ++i){
                found = result[i].equalsIgnoreCase(test);
            }
            assertTrue("Can't find " + test + " in result.", found);

        }
    }

    @Test
    public void test_searchFiles(){
        PackageMojo pm = new PackageMojo();
        Set<String> testSet = new HashSet<String>();
        testSet.add("ema");
        testSet.add("emam");
        testSet.add("stream");
        testSet.add("struct");
        testSet.add("enum");
        pm.setFilesToPack(String.join(",", testSet));
    }

    @Test
    public void test_execute_noMain(){
        PackageMojo pm = new PackageMojo();
        pm.setFilesToPack("ema, emam, stream, struct, enum");
        pm.setPathMain("./src/test/emam/main");
        pm.setPathOut("./target/test");
        pm.setPackageName("test_noMain");
        try {
            pm.execute();
        } catch (MojoExecutionException e) {
            e.printStackTrace();
        } catch (MojoFailureException e) {
            e.printStackTrace();
        }catch (Exception ex){
            ex.printStackTrace();
            fail("Exception in execute");
        }

        File f = new File("./target/test/test_noMain.zip");
        assertTrue("PackageMojo hasn't created a zip file.", f.exists());

        try {
            File noMainDir = new File("./target/test/test_noMain");
            if(noMainDir.exists()){
                noMainDir.delete();
            }
            noMainDir.mkdirs();
            ZipFileCreator.ExtractTo("./target/test/test_noMain.zip", "./target/test/test_noMain");
        } catch (IOException e) {
            e.printStackTrace();
            fail("Error while reading zip.");
        }

        Map<String,File> filesIn = SearchFiles.searchFilesMap("./src/test/emam/main/", "ema", "emam", "stream", "struct", "enum");
        Map<String,File> filesOut = SearchFiles.searchFilesMap("./target/test/test_noMain/", "ema", "emam", "stream", "struct", "enum");

        for (Map.Entry<String,File> inF:filesIn.entrySet()) {
            assertTrue("Can't find file "+inF.getKey(), filesOut.containsKey(inF.getKey()));
            try {
                assertTrue("Files not equal "+inF.getKey(), FileUtils.contentEquals(inF.getValue(), filesOut.get(inF.getKey())));
            } catch (IOException e) {
                e.printStackTrace();
                fail("Error while reading file(s) "+inF.getKey());
            }
        }
    }
}