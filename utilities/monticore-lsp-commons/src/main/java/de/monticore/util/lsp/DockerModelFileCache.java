/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import de.se_rwth.commons.logging.Log;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.Set;

public class DockerModelFileCache extends ModelFileCache {
    private Options options;

    public DockerModelFileCache(Set<String> fileExtensions, Options options) {
        super(fileExtensions);
        this.options = options;
    }

    @Override
    public void addModelBasePath(@NotNull VSCodeUri originalPath, List<String> packageList) {
        super.addModelBasePath(translateUri(originalPath), packageList);
    }

    @Override
    public Optional<VSCodeUri> fromTmpPath(Path path) {
        String externalPathStr = String.join("/", options.externalPath);
        String internalPathStr = String.join("/", options.internalPath);
        return super.fromTmpPath(path).map(uri -> translateUri(uri, externalPathStr, internalPathStr));
    }

    @Override
    public void updateTmpContentFor(VSCodeUri originalUri, String content) throws IOException {
        super.updateTmpContentFor(translateUri(originalUri), content);
    }

    @Override
    public Optional<String> getCachedContentFor(VSCodeUri originalUri) {
        return super.getCachedContentFor(translateUri(originalUri));
    }

    @Override
    public Path getTmpModelPath(VSCodeUri orignalUri) {
        return super.getTmpModelPath(translateUri(orignalUri));
    }

    protected VSCodeUri translateUri(VSCodeUri originalUri){
        String externalUriStr = "file:///" + String.join("/", options.externalPath);
        String internalPathStr = "file:///" + String.join("/", options.internalPath);
        return translateUri(originalUri, internalPathStr, externalUriStr);
    }

    @NotNull
    private VSCodeUri translateUri(VSCodeUri originalUri, String internalPathStr, String externalUriStr) {
        String originalPathStr = originalUri.toString();
        if(originalPathStr.startsWith(externalUriStr)){
            String resStr = internalPathStr + originalPathStr.substring(externalUriStr.length());
            try {
                Log.trace(String.format("Translated %s to %s", originalUri.toString() ,resStr), "translateUri");
                return new VSCodeUri(resStr);
            } catch (URISyntaxException e) {
                Log.error("Error in translateUri " + originalUri, e);
            }
            return originalUri;
        }

        Log.warn("Can not convert " + originalUri + " because external path " + externalUriStr + " is not a prefix. Using original path as fallback!");
        return originalUri;
    }

    public static class Options{
        private List<String> externalPath;
        private List<String> internalPath;

        public Options(List<String> externalPath, List<String> internalPath) {
            this.externalPath = externalPath;
            this.internalPath = internalPath;
        }

        @Override
        public String toString() {
            return "Options{" +
                    "externalPath=" + externalPath +
                    ", internalPath=" + internalPath +
                    '}';
        }
    }
}
