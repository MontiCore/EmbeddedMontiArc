/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.io.IOUtils;
import org.eclipse.lsp4j.TextDocumentIdentifier;
import org.eclipse.lsp4j.TextDocumentItem;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

public class ModelFileCache {
    private BiMap<Path, Path> modelBaseToTmpPath;
    private Set<String> fileExtensions;

    public ModelFileCache(@Nullable Path modelBasePath, Set<String> fileExtensions){
        this.modelBaseToTmpPath = HashBiMap.create();
        this.fileExtensions = fileExtensions;
        addModelBasePath(modelBasePath);
    }

    public ModelFileCache(Set<String> fileExtensions){
        this(null, fileExtensions);
    }

    public void addModelBasePath(@Nullable Path modelBasePath) {
        if (modelBasePath != null) {
            Path normalized = modelBasePath.normalize();
            if (!modelBaseToTmpPath.containsKey(normalized)) {
                try {
                    Path tempDir = Files.createTempDirectory("model");
                    modelBaseToTmpPath.put(normalized, tempDir);
                    initialize(normalized, tempDir);
                } catch (IOException e) {
                    Log.error("Error creating temp dir", e);
                }
            }
        }
    }

    public Optional<Path> convertToTmpPath(Path path){
        return convertPaths(path, modelBaseToTmpPath);
    }

    public Optional<Path> fromTmpPath(Path path){
        return convertPaths(path, modelBaseToTmpPath.inverse());
    }

    private Optional<Path> convertPaths(Path path, Map<Path, Path> pathMap){
        Path normalize = path.normalize();
        Optional<Path> bestBasePath = getMostSpecificPathContaining(normalize, pathMap.keySet());

        if(bestBasePath.isPresent()) {
            Path tmpModelPath = pathMap.get(bestBasePath.get());
            Optional<Path> rest = bestBasePath.flatMap(p -> ModelPathHelper.getPathRelativeTo(p, path));
            return rest.map(tmpModelPath::resolve);
        }

        return Optional.empty();
    }

    @NotNull
    private Optional<Path> getMostSpecificPathContaining(@NotNull Path path, @NotNull Collection<Path> candidatePaths) {
        return candidatePaths
                    .stream()
                    .filter(path::startsWith)
                    .max(Comparator.comparing(Path::getNameCount));
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

    public Path getTmpModelPath(Path basePath){
        Optional<Path> mostSpecificPathContaining = getMostSpecificPathContaining(basePath, modelBaseToTmpPath.keySet());
        if(mostSpecificPathContaining.isPresent()){
            return modelBaseToTmpPath.get(mostSpecificPathContaining.get());
        }
        return basePath;
    }

    public void initialize(@NotNull Path modelBasePath, Path tmpModelPath) throws IOException {
        FileUtils.copyDirectory(modelBasePath.toFile(), tmpModelPath.toFile(), file -> {
            if (file.isDirectory()) {
                return true;
            }
            String extension = FilenameUtils.getExtension(file.getName());
            return fileExtensions.contains(extension);
        });
    }
}
