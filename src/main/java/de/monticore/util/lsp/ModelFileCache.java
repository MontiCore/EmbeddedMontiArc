/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.io.IOUtils;
import org.eclipse.lsp4j.TextDocumentIdentifier;
import org.eclipse.lsp4j.TextDocumentItem;
import org.jetbrains.annotations.Nullable;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

public class ModelFileCache {
    private Path modelBasePath;
    private Path tmpModelPath;
    private Set<String> fileExtensions;
    private boolean initialized = false;

    public ModelFileCache(Path modelBasePath, Set<String> fileExtensions) throws IOException {
        this.modelBasePath = modelBasePath;
        this.tmpModelPath = Files.createTempDirectory("model");
        this.fileExtensions = fileExtensions;
    }

    public ModelFileCache(Set<String> fileExtensions) throws IOException {
        this(null, fileExtensions);
    }

    public void setModelBasePath(@Nullable Path modelBasePath) {
        if(!Objects.equals(modelBasePath, this.modelBasePath)) {
            this.modelBasePath = modelBasePath;
            try {
                initialize();
            } catch (IOException e) {
                Log.error("Error initializing", e);
            }
        }
    }

    public boolean isInitialized() {
        return initialized;
    }

    public void setInitialized(boolean initialized) {
        this.initialized = initialized;
    }

    public Optional<Path> convertToTmpPath(Path path){
        if(modelBasePath == null){
            return Optional.ofNullable(path);
        }

        Optional<Path> rest = ModelPathHelper.getPathRelativeTo(modelBasePath, path);
        return rest.map(tmpModelPath::resolve);
    }

    public Optional<Path> fromTmpPath(Path path){
        if(modelBasePath == null){
            return Optional.ofNullable(path);
        }

        Optional<Path> rest = ModelPathHelper.getPathRelativeTo(tmpModelPath, path);
        return rest.map(modelBasePath::resolve);
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

    public Optional<String> getCachedContentFor(String uriString){
        try {
            Path originalPath = ModelPathHelper.pathFromUriString(uriString);
            return convertToTmpPath(originalPath)
                    .flatMap(p -> {
                        try {
                            return Optional.of(IOUtils.toString(p.toUri(), StandardCharsets.UTF_8));
                        } catch (IOException e) {
                            Log.error("Error loading file " + p.toUri(), e);
                        }
                        return Optional.empty();
                    });
        } catch (URISyntaxException e) {
            Log.error("Error in getCachedContentFor(" +  uriString + ")", e);
        }

        return Optional.empty();
    }

    public Optional<String> getCachedContentFor(TextDocumentIdentifier textDocumentIdentifier){
        return getCachedContentFor(textDocumentIdentifier.getUri());
    }

    public Optional<String> getCachedContentFor(TextDocumentItem textDocumentItem){
        return getCachedContentFor(textDocumentItem.getUri());
    }

    public Path getTmpModelPath(){
        return tmpModelPath;
    }

    public void initialize() throws IOException {
        if(modelBasePath != null) {
            FileUtils.copyDirectory(modelBasePath.toFile(), tmpModelPath.toFile(), file -> {
                if (file.isDirectory()) {
                    return true;
                }
                String extension = FilenameUtils.getExtension(file.getName());
                return fileExtensions.contains(extension);
            });

            initialized = true;
        }else{
            initialized = false;
        }
    }


}
