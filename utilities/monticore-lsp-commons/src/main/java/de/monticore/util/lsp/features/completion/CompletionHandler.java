/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.features.completion;

import org.eclipse.lsp4j.CompletionItem;
import org.eclipse.lsp4j.CompletionList;
import org.eclipse.lsp4j.CompletionParams;
import org.eclipse.lsp4j.jsonrpc.messages.Either;

import java.util.List;
import java.util.concurrent.CompletableFuture;

public interface CompletionHandler {
    CompletableFuture<Either<List<CompletionItem>, CompletionList>> completion(CompletionParams position);
}
