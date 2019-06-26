package de.monticore.util.lsp;

import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.FilenameUtils;

import java.io.File;
import java.io.FileFilter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.Optional;
import java.util.Set;

public class ModelFileCache {
    private Path modelBasePath;
    private Path tmpModelPath;
    private Set<String> fileExtensions;

    public ModelFileCache(Path modelBasePath, Set<String> fileExtensions) throws IOException {
        this.modelBasePath = modelBasePath;
        this.tmpModelPath = Files.createTempDirectory("model");
        this.fileExtensions = fileExtensions;
        this.copyModel();
    }

    public Optional<Path> convertToTmpPath(Path path){
        Optional<Path> rest = ModelPathHelper.getPathRelativeTo(modelBasePath, path);
        return rest.map(tmpModelPath::resolve);
    }

    public void updateTmpContentFor(Path path, String content) throws IOException {
        Log.debug("Changing content of " + path, "files");
        Optional<Path> tmpPath = convertToTmpPath(path);
        if(tmpPath.isPresent()){
            Log.debug("Resolved file: " + tmpPath.get(), "files");
            Files.write(tmpPath.get(), Collections.singleton(content));
        }else{
            Log.debug("Can not find tmp path for " + path, "files");
        }

    }

    public Optional<Path> fromTmpPath(Path path){
        Optional<Path> rest = ModelPathHelper.getPathRelativeTo(tmpModelPath, path);
        return rest.map(modelBasePath::resolve);
    }

    public Path getTmpModelPath(){
        return tmpModelPath;
    }

    private void copyModel() throws IOException {
        FileUtils.copyDirectory(modelBasePath.toFile(), tmpModelPath.toFile(), new FileFilter() {
            @Override
            public boolean accept(File file) {
                if (file.isDirectory()) {
                    return true;
                }
                String extension = FilenameUtils.getExtension(file.getName());
                return fileExtensions.contains(extension);
            }
        });
    }


}
