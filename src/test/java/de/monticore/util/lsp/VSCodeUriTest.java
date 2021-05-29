/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import com.google.common.collect.Lists;
import de.se_rwth.commons.logging.DiagnosticsLog;
import de.se_rwth.commons.logging.Log;
import org.jetbrains.annotations.NotNull;
import org.junit.Before;
import org.junit.Test;

import java.net.URISyntaxException;
import java.util.List;

import static org.junit.Assert.assertEquals;

public class VSCodeUriTest {

    @Before
    public void initLog(){
        DiagnosticsLog.init();
        DiagnosticsLog.setLogToStdout(true);
        DiagnosticsLog.setTrace(true);
        DiagnosticsLog.setDebug(true);
        Log.getFindings().clear();
    }

    @NotNull
    private VSCodeUri getWinUri() throws URISyntaxException {
        String uriString = "file:///c%3A/Users/username/montibelle/ive/res/test/refinementRoot/spec/Refined.arc";
        return new VSCodeUri(uriString);
    }

    @NotNull
    private VSCodeUri getLinUri() throws URISyntaxException {
        String uriString = "file:///home/username/montibelle/ive/res/test/refinementRoot/spec/Refined.arc";
        return new VSCodeUri(uriString);
    }

    @Test
    public void testWin() throws URISyntaxException {
        VSCodeUri uri = new VSCodeUri("C:\\Users\\username\\montibelle\\ive\\res\\test\\refinementRoot\\spec\\Refined.arc");
        assertEquals("file:///C:/Users/username/montibelle/ive/res/test/refinementRoot/spec/Refined.arc", uri.toString());
    }

    @Test
    public void testToStringWin() throws URISyntaxException {
        VSCodeUri uri = getWinUri();
        assertEquals("file:///C:/Users/username/montibelle/ive/res/test/refinementRoot/spec/Refined.arc", uri.toString());
    }

    @Test
    public void testToStringLin() throws URISyntaxException {
        VSCodeUri uri = getLinUri();
        assertEquals("file:///home/username/montibelle/ive/res/test/refinementRoot/spec/Refined.arc", uri.toString());
    }

    @Test
    public void getFsPathPartsWin() throws URISyntaxException {
        VSCodeUri uri = getWinUri();

        List<String> res = Lists.asList("C:", new String[]{"Users", "username", "montibelle", "ive", "res", "test", "refinementRoot", "spec", "Refined.arc"});
        assertEquals(res, uri.getFsPathParts());
    }

    @Test
    public void getFsPathPartsLin() throws URISyntaxException {
        VSCodeUri uri = getLinUri();

        List<String> res = Lists.asList("home", new String[]{"username", "montibelle", "ive", "res", "test", "refinementRoot", "spec", "Refined.arc"});
        assertEquals(res, uri.getFsPathParts());
    }
}
