/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import de.se_rwth.commons.logging.Log;
import org.eclipse.lsp4j.TextDocumentIdentifier;
import org.eclipse.lsp4j.TextDocumentItem;
import org.jetbrains.annotations.NotNull;

import java.net.URI;
import java.net.URISyntaxException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class VSCodeUri {
    // needed since URI is final
    private URI delegate;

    public VSCodeUri(URI uri){
        delegate = uri;
    }

    public VSCodeUri(Path path) {
        this(path.toUri());
    }

    public Path getFsPath(){
        String pathStr = delegate.getPath();
        if(pathStr.contains(":") && pathStr.startsWith("/")){
            pathStr = pathStr.substring(1);
        }
        Log.trace(pathStr, "fsPath");
        return Paths.get(pathStr);
    }

    public URI asUri(){
        return delegate;
    }

    public List<String> getFsPathParts(){
        String s = delegate.getPath();
        if(s.startsWith("/")){
            return Arrays.asList(s.substring(1).split("/"));
        }else{
            return Arrays.asList(s.split("/"));
        }
    }

    public String getEncodedString() {
        return ModelPathHelper.encodePathStringToUri(this.toString());
    }

    public VSCodeUri(@NotNull String str) throws URISyntaxException {
        this(new URI(ModelPathHelper.cleanVSCodeUriString(str)));
    }

    public VSCodeUri(@NotNull TextDocumentIdentifier textDocumentIdentifier) throws URISyntaxException {
        this(textDocumentIdentifier.getUri());
    }

    public VSCodeUri(@NotNull TextDocumentItem textDocument) throws URISyntaxException {
        this(textDocument.getUri());
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null) return false;
        if(o instanceof URI){
            return o.equals(delegate);
        }

        if(o instanceof VSCodeUri) {
            VSCodeUri vsCodeUri = (VSCodeUri) o;
            return Objects.equals(delegate, vsCodeUri.delegate);
        }else{
            return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(delegate);
    }

    @Override
    public String toString() {
        return delegate.toString();
    }

}
