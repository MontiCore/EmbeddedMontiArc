package de.monticore.util.lsp.features.completion;

import de.monticore.util.lsp.ModelFileCache;
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
    private ModelFileCache cache;
    private LookaheadProvider lookaheadProvider;

    public DefaultCompletionHandler(ModelFileCache cache, LookaheadProvider lookaheadProvider) {
        this.cache = cache;
        this.lookaheadProvider = lookaheadProvider;
    }

    public CompletableFuture<Either<List<CompletionItem>, CompletionList>> completion(CompletionParams position) {
        Optional<String> contentOpt = cache.getCachedContentFor(position.getTextDocument().getUri());
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