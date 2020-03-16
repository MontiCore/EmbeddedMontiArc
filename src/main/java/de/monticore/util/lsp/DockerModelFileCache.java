package de.monticore.util.lsp;

import de.se_rwth.commons.logging.Log;
import org.jetbrains.annotations.Nullable;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;
import java.util.Set;

public class DockerModelFileCache extends ModelFileCache {
    private Path externalPath;
    private Path internalPath;

    public DockerModelFileCache(@Nullable Path modelBasePath, Set<String> fileExtensions, Path externalPath, Path internalPath) {
        super(fileExtensions);
        this.internalPath = internalPath;
        this.externalPath = externalPath;
        if(modelBasePath != null){
            addModelBasePath(modelBasePath);
        }
    }

    @Override
    public void addModelBasePath(@Nullable Path modelBasePath) {
        super.addModelBasePath(translatePath(modelBasePath));
    }

    @Override
    public Optional<Path> convertToTmpPath(Path path) {
        return super.convertToTmpPath(translatePath(path));
    }

    @Override
    public Optional<Path> fromTmpPath(Path path) {
        return super.fromTmpPath(translatePath(path));
    }

    @Override
    public Path getTmpModelPath(Path basePath) {
        return super.getTmpModelPath(translatePath(basePath));
    }

    @Override
    public void updateTmpContentFor(Path path, String content) throws IOException {
        super.updateTmpContentFor(translatePath(path), content);
    }

    public Path translatePath(Path path){
        if(path.startsWith(externalPath)){
            Path rest = path.subpath(externalPath.getNameCount(), path.getNameCount());
            return internalPath.resolve(rest);
        }

        Log.warn("Can not convert " + path + " because external path " + externalPath + " is not a prefix. Using original path as fallback!");
        return path;
    }
}
