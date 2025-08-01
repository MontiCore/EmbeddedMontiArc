/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.features.completion;

import de.se_rwth.commons.logging.Log;
import org.eclipse.lsp4j.CompletionItem;
import org.eclipse.lsp4j.CompletionList;
import org.eclipse.lsp4j.CompletionParams;
import org.eclipse.lsp4j.jsonrpc.messages.Either;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

public class MultiCompletionHandler implements CompletionHandler {
    private List<CompletionHandler> delegates = new ArrayList<>();

    public MultiCompletionHandler(List<CompletionHandler> delegates) {
        this.delegates.addAll(delegates);
    }

    public MultiCompletionHandler(CompletionHandler... delegates) {
        this.delegates.addAll(Arrays.asList(delegates));
    }

    public List<CompletionHandler> getDelegates() {
        return delegates;
    }

    public void setDelegates(List<CompletionHandler> delegates) {
        this.delegates = delegates;
    }

    public boolean add(CompletionHandler completionHandler) {
        return delegates.add(completionHandler);
    }

    @Override
    public CompletableFuture<Either<List<CompletionItem>, CompletionList>> completion(CompletionParams position) {
        CompletionList res = new CompletionList();
        for (CompletionHandler delegate : delegates) {
            try {
                Either<List<CompletionItem>, CompletionList> curRes = delegate.completion(position).get();
                if(curRes != null){
                    if(curRes.isLeft()){
                        res.getItems().addAll(curRes.getLeft());
                    }else{
                        CompletionList cl = curRes.getRight();
                        res.getItems().addAll(cl.getItems());
                        res.setIsIncomplete(res.isIncomplete() || cl.isIncomplete());
                    }
                }
            } catch (InterruptedException | ExecutionException e) {
                Log.error("Error while execution completion request!", e);
            }
        }

        return CompletableFuture.completedFuture(Either.forRight(res));
    }
}
