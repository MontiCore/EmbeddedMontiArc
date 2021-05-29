/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import de.se_rwth.commons.logging.DiagnosticsLog;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.net.URISyntaxException;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assume.assumeTrue;

public class DockerModelFileCacheTest {

    public static final List<String> EXTERNAL_PATH_WIN = Arrays.asList("C:\\path\\to\\external\\dir".split("[\\\\]"));
    public static final List<String> EXTERNAL_PATH_LINUX = Arrays.asList("path/to/external/dir".split("/"));
    public static final List<String> INTERNAL_PATH_LINUX = Arrays.asList("path/to/internal/dir".split("/"));
    public static final DockerModelFileCache WIN_TO_LIN_CACHE = new DockerModelFileCache(null, new DockerModelFileCache.Options(EXTERNAL_PATH_WIN, INTERNAL_PATH_LINUX));
    public static final DockerModelFileCache LIN_TO_LIN_CACHE = new DockerModelFileCache(null, new DockerModelFileCache.Options(EXTERNAL_PATH_LINUX, INTERNAL_PATH_LINUX));

    @Before
    public void initLog(){
        DiagnosticsLog.init();
        DiagnosticsLog.setLogToStdout(true);
        DiagnosticsLog.setTrace(true);
        DiagnosticsLog.setDebug(true);
        Log.getFindings().clear();
    }

    @Test
    public void translatePathSimpleWindows() throws URISyntaxException {
        assumeTrue(System.getProperty("os.name").toLowerCase().startsWith("windows"));

        VSCodeUri res = WIN_TO_LIN_CACHE.translateUri(new VSCodeUri(new File("C:\\path\\to\\external\\dir\\file.txt").toPath()));
        assertEquals(new VSCodeUri("file:///path/to/internal/dir/file.txt"), res);
    }

    @Test
    public void translatePathNestedWindows() throws URISyntaxException {
        assumeTrue(System.getProperty("os.name").toLowerCase().startsWith("windows"));

        VSCodeUri res = WIN_TO_LIN_CACHE.translateUri(new VSCodeUri(new File("C:\\path\\to\\external\\dir\\nested\\file.txt").toPath()));
        assertEquals(new VSCodeUri("file:///path/to/internal/dir/nested/file.txt"), res);
    }

    @Test
    public void translatePathSimpleLinux() throws URISyntaxException {
        VSCodeUri res = LIN_TO_LIN_CACHE.translateUri(new VSCodeUri("file:///path/to/external/dir/file.txt"));
        assertEquals(new VSCodeUri("file:///path/to/internal/dir/file.txt"), res);
    }

    @Test
    public void translatePathNestedLinux() throws URISyntaxException {
        VSCodeUri res = LIN_TO_LIN_CACHE.translateUri(new VSCodeUri("file:///path/to/external/dir/nested/file.txt"));
        assertEquals(new VSCodeUri("file:///path/to/internal/dir/nested/file.txt"), res);
    }
}
