/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.File;
import java.net.URISyntaxException;

import static org.junit.Assert.*;

public class ModelPathHelperTest {

    @BeforeClass
    public static void beforeClass(){
        Log.init();
    }

    @Test
    public void pathFromUriStringWin() throws URISyntaxException {
        String expected = String.join(File.separator, "C:", "Users", "Test", "Desktop", "workspace");
        String actual = ModelPathHelper.pathFromUriString("file:///c%3a/Users/Test/Desktop/workspace").toString();
        assertEquals(expected, actual);
    }

    @Test
    public void pathFromUriStringUnix() throws URISyntaxException {
        String expected = File.separator + String.join(File.separator, "home", "Test", "Desktop", "workspace");
        String actual = ModelPathHelper.pathFromUriString("file:///home/Test/Desktop/workspace").toString();
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
