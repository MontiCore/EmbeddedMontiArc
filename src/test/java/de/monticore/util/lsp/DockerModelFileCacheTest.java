package de.monticore.util.lsp;

import org.junit.Test;

import java.io.File;
import java.nio.file.Path;

import static org.junit.Assert.assertEquals;
import static org.junit.Assume.assumeTrue;

public class DockerModelFileCacheTest {

    @Test
    public void translatePathSimpleWindows() {
        assumeTrue(System.getProperty("os.name").toLowerCase().contains("windows"));

        DockerModelFileCache cache = new DockerModelFileCache(null, null, new File("C:\\path\\to\\external\\dir").toPath(), new File("/path/to/internal/dir").toPath());
        Path res = cache.translatePath(new File("C:\\path\\to\\external\\dir\\file.txt").toPath());
        assertEquals(new File("/path/to/internal/dir/file.txt").toPath(), res);
    }

    @Test
    public void translatePathNestedWindows() {
        assumeTrue(System.getProperty("os.name").toLowerCase().contains("windows"));

        DockerModelFileCache cache = new DockerModelFileCache(null, null, new File("C:\\path\\to\\external\\dir").toPath(), new File("/path/to/internal/dir").toPath());
        Path res = cache.translatePath(new File("C:\\path\\to\\external\\dir\\nested\\file.txt").toPath());
        assertEquals(new File("/path/to/internal/dir/nested/file.txt").toPath(), res);
    }

    @Test
    public void translatePathSimpleLinux() {
        DockerModelFileCache cache = new DockerModelFileCache(null, null, new File("/path/to/external/dir").toPath(), new File("/path/to/internal/dir").toPath());
        Path res = cache.translatePath(new File("/path/to/external/dir/file.txt").toPath());
        assertEquals(new File("/path/to/internal/dir/file.txt").toPath(), res);
    }

    @Test
    public void translatePathNestedLinux() {
        DockerModelFileCache cache = new DockerModelFileCache(null, null, new File("/path/to/external/dir").toPath(), new File("/path/to/internal/dir").toPath());
        Path res = cache.translatePath(new File("/path/to/external/dir/nested/file.txt").toPath());
        assertEquals(new File("/path/to/internal/dir/nested/file.txt").toPath(), res);
    }
}