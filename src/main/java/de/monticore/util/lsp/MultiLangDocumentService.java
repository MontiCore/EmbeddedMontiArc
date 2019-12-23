package de.monticore.util.lsp;

import de.monticore.util.lsp.exceptions.FileExtensionAlreadyRegisteredException;
import org.eclipse.lsp4j.*;
import org.eclipse.lsp4j.jsonrpc.messages.Either;
import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.lsp4j.services.TextDocumentService;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;

public class MultiLangDocumentService implements ClientAwareTextDocumentService {
    private Map<String, ClientAwareTextDocumentService> fileExtensionToDocumentServices = new HashMap<>();

    public void addDocumentService(String fileExtension, ClientAwareTextDocumentService documentService) throws FileExtensionAlreadyRegisteredException {
        String fileExtensionCleaned = fileExtension.replace(".", "");
        if(fileExtensionToDocumentServices.containsKey(fileExtensionCleaned)){
            throw new FileExtensionAlreadyRegisteredException("This file extension is already registered: " + fileExtensionCleaned);
        }
        fileExtensionToDocumentServices.put(fileExtensionCleaned, documentService);
    }

    private String fileExtensionFromUri(@NotNull String uri){
        String[] split = uri.split("[.]");
        if(split.length > 0) {
            return split[split.length - 1];
        }else{
            return "";
        }
    }

    private @Nullable TextDocumentService getMatchingDocumentService(@NotNull TextDocumentIdentifier id){
        String fileExtension = fileExtensionFromUri(id.getUri());
        return fileExtensionToDocumentServices.getOrDefault(fileExtension, null);
    }

    private @Nullable TextDocumentService getMatchingDocumentService(@NotNull String uri){
        String fileExtension = fileExtensionFromUri(uri);
        return fileExtensionToDocumentServices.getOrDefault(fileExtension, null);
    }

    @Override
    public CompletableFuture<WorkspaceEdit> rename(RenameParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.rename(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public void didOpen(DidOpenTextDocumentParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument().getUri());
        if(documentService != null){
            documentService.didOpen(arg);
        }
    }

    @Override
    public void didChange(DidChangeTextDocumentParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            documentService.didChange(arg);
        }
    }

    @Override
    public void didClose(DidCloseTextDocumentParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            documentService.didClose(arg);
        }
    }

    @Override
    public void didSave(DidSaveTextDocumentParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            documentService.didSave(arg);
        }
    }

    @Override
    public void willSave(WillSaveTextDocumentParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            documentService.willSave(arg);
        }
    }

    @Override
    public CompletableFuture<Either<List<CompletionItem>, CompletionList>> completion(CompletionParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.completion(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<Hover> hover(TextDocumentPositionParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.hover(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<SignatureHelp> signatureHelp(TextDocumentPositionParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.signatureHelp(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<Either<List<? extends Location>, List<? extends LocationLink>>> declaration(TextDocumentPositionParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.declaration(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<Either<List<? extends Location>, List<? extends LocationLink>>> definition(TextDocumentPositionParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.definition(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<Either<List<? extends Location>, List<? extends LocationLink>>> typeDefinition(TextDocumentPositionParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.typeDefinition(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<Either<List<? extends Location>, List<? extends LocationLink>>> implementation(TextDocumentPositionParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.implementation(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<? extends Location>> references(ReferenceParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.references(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<? extends DocumentHighlight>> documentHighlight(TextDocumentPositionParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.documentHighlight(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<Either<SymbolInformation, DocumentSymbol>>> documentSymbol(DocumentSymbolParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.documentSymbol(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<Either<Command, CodeAction>>> codeAction(CodeActionParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.codeAction(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<? extends CodeLens>> codeLens(CodeLensParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.codeLens(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }


    @Override
    public CompletableFuture<List<? extends TextEdit>> formatting(DocumentFormattingParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.formatting(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<? extends TextEdit>> rangeFormatting(DocumentRangeFormattingParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.rangeFormatting(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<? extends TextEdit>> onTypeFormatting(DocumentOnTypeFormattingParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.onTypeFormatting(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<TextEdit>> willSaveWaitUntil(WillSaveTextDocumentParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.willSaveWaitUntil(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<DocumentLink>> documentLink(DocumentLinkParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.documentLink(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<ColorInformation>> documentColor(DocumentColorParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.documentColor(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<ColorPresentation>> colorPresentation(ColorPresentationParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.colorPresentation(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<List<FoldingRange>> foldingRange(FoldingRangeRequestParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.foldingRange(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<Either<Range, PrepareRenameResult>> prepareRename(TextDocumentPositionParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.prepareRename(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<TypeHierarchyItem> typeHierarchy(TypeHierarchyParams arg){

        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.typeHierarchy(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public CompletableFuture<CallHierarchyItem> callHierarchy(CallHierarchyParams arg){
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if(documentService != null){
            return documentService.callHierarchy(arg);
        }else{
            return CompletableFuture.completedFuture(null);
        }
    }

    @Override
    public void setClient(LanguageClient client) {
        connect(client);
    }

    @Override
    public void connect(LanguageClient languageClient) {
        fileExtensionToDocumentServices.forEach((key, value) -> value.connect(languageClient));
    }
}
