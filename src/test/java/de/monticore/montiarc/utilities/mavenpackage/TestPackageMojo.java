package de.monticore.montiarc.utilities.mavenpackage;

import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.junit.Test;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class TestPackageMojo {

    @Test
    public void test_execute(){
        PackageMojo pm = new PackageMojo();
        try {
            pm.execute();
        } catch (MojoExecutionException e) {
            e.printStackTrace();
        } catch (MojoFailureException e) {
            e.printStackTrace();
        }
    }

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
}