/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Test;

import java.net.URISyntaxException;
import java.nio.file.Path;
import java.nio.file.Paths;

import static org.junit.Assert.assertEquals;
import static org.junit.Assume.assumeFalse;
import static org.junit.Assume.assumeTrue;

public class ModelPathHelperTest {

    @BeforeClass
    public static void beforeClass(){
        Log.init();
    }

    @Test
    public void pathFromUriStringWin() throws URISyntaxException {
        assumeTrue(System.getProperty("os.name").toLowerCase().startsWith("windows"));

        Path expected = Paths.get("C:", "Users", "Test", "Desktop", "workspace");
        Path actual = ModelPathHelper.pathFromUriString("file:///c%3a/Users/Test/Desktop/workspace");
        assertEquals(expected, actual);
    }

    @Test
    public void pathFromUriStringUnix() throws URISyntaxException {
        assumeFalse(System.getProperty("os.name").toLowerCase().startsWith("windows"));

        Path expected = Paths.get("/","home", "Test", "Desktop", "workspace");
        Path actual = ModelPathHelper.pathFromUriString("file:///home/Test/Desktop/workspace");
        assertEquals(expected, actual);
    }

    @Test
    public void encodePathStringToUriWindows(){
        String expected = "file:///c%3a/Users/Test/Desktop/workspace";
        String actual = ModelPathHelper.encodePathStringToUri("C:\\Users\\Test\\Desktop\\workspace");
        assertEquals(expected, actual);
    }

    @Test
    public void encodePathStringToUriUnix(){
        String expected = "file:///home/Test/Desktop/workspace";
        String actual = ModelPathHelper.encodePathStringToUri("/home/Test/Desktop/workspace");
        assertEquals(expected, actual);
    }
}
