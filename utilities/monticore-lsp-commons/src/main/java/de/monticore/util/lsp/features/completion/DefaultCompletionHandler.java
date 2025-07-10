/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.features.completion;

import de.monticore.util.lsp.util.ModelFileCacheProvider;
import org.eclipse.lsp4j.CompletionItem;
import org.eclipse.lsp4j.CompletionList;
import org.eclipse.lsp4j.CompletionParams;
import org.eclipse.lsp4j.Position;
import org.eclipse.lsp4j.jsonrpc.messages.Either;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;

public class DefaultCompletionHandler implements CompletionHandler{
    private ModelFileCacheProvider cacheProvider;
    private LookaheadProvider lookaheadProvider;

    public DefaultCompletionHandler(ModelFileCacheProvider cacheProvider, LookaheadProvider lookaheadProvider) {
        this.cacheProvider = cacheProvider;
        this.lookaheadProvider = lookaheadProvider;
    }

    public CompletableFuture<Either<List<CompletionItem>, CompletionList>> completion(CompletionParams position) {
        Optional<String> contentOpt = cacheProvider.getModelFileCache().getCachedContentFor(position.getTextDocument());
        if(contentOpt.isPresent()){
            Position p = position.getPosition();
            Optional<LookaheadContext> lookahead = lookaheadProvider.getLookaheadFor(contentOpt.get(), p.getLine(), p.getCharacter());

            List<CompletionItem> res = new ArrayList<>();

            lookahead.ifPresent(lookaheadContext -> lookaheadContext
                    .getStringTokens()
                    .stream()
                    .map(CompletionItem::new)
                    .forEach(res::add)
            );

            return CompletableFuture.completedFuture(Either.forLeft(res));
        }

        return CompletableFuture.completedFuture(Either.forLeft(new ArrayList<>()));
    }
}
