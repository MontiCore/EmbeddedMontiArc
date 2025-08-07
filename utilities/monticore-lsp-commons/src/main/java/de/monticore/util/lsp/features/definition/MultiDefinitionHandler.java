/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.features.definition;

import de.se_rwth.commons.logging.Log;
import org.eclipse.lsp4j.Location;
import org.eclipse.lsp4j.LocationLink;
import org.eclipse.lsp4j.TextDocumentPositionParams;
import org.eclipse.lsp4j.jsonrpc.messages.Either;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

public class MultiDefinitionHandler implements DefinitionHandler {
    private List<DefinitionHandler> delegates = new ArrayList<>();

    public MultiDefinitionHandler(List<DefinitionHandler> delegates) {
        this.delegates = delegates;
    }

    public MultiDefinitionHandler(DefinitionHandler... delegates) {
        this.delegates.addAll(Arrays.asList(delegates));
    }

    public List<DefinitionHandler> getDelegates() {
        return delegates;
    }

    public boolean add(DefinitionHandler definitionHandler) {
        return delegates.add(definitionHandler);
    }

    @Override
    public CompletableFuture<Either<List<? extends Location>, List<? extends LocationLink>>> definition(TextDocumentPositionParams position) {
        List<Location> locs = new ArrayList<>();
        List<LocationLink> locLinks = new ArrayList<>();

        for(DefinitionHandler d : delegates){
            try {
                Either<List<? extends Location>, List<? extends LocationLink>> res = d.definition(position).get();
                if(res != null) {
                    if (res.isLeft() && res.getLeft() != null) {
                        locs.addAll(res.getLeft());
                    } else if(res.isRight() && res.getRight() != null){
                        locLinks.addAll(res.getRight());
                    }
                }
            } catch (InterruptedException | ExecutionException e) {
                e.printStackTrace();
            }
        }

        if(locs.size() > 0 && locLinks.size() > 0){
            Log.warn("Received locations and location links from the DefinitionHandlers.");
            if(locs.size() >= locLinks.size()){
                Log.warn("Will ignore location links!");
                return CompletableFuture.completedFuture(Either.forLeft(locs));
            }else{
                Log.warn("Will ignore locations!");
                return CompletableFuture.completedFuture(Either.forRight(locLinks));
            }
        }

        if(locs.size() > 0){
            return CompletableFuture.completedFuture(Either.forLeft(locs));
        }

        if(locLinks.size() > 0){
            return CompletableFuture.completedFuture(Either.forRight(locLinks));
        }

        return CompletableFuture.completedFuture(Either.forLeft(new ArrayList<>()));
    }
}
