/* (c) https://github.com/MontiCore/monticore */
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
        if (fileExtensionToDocumentServices.containsKey(fileExtensionCleaned)) {
            throw new FileExtensionAlreadyRegisteredException("This file extension is already registered: " + fileExtensionCleaned);
        }
        fileExtensionToDocumentServices.put(fileExtensionCleaned, documentService);
    }

    private String fileExtensionFromUri(@NotNull String uri) {
        String[] split = uri.split("[.]");
        if (split.length > 0) {
            return split[split.length - 1];
        } else {
            return "";
        }
    }

    private @Nullable TextDocumentService getMatchingDocumentService(@NotNull TextDocumentIdentifier id) {
        String fileExtension = fileExtensionFromUri(id.getUri());
        return fileExtensionToDocumentServices.getOrDefault(fileExtension, null);
    }

    private @Nullable TextDocumentService getMatchingDocumentService(@NotNull String uri) {
        String fileExtension = fileExtensionFromUri(uri);
        return fileExtensionToDocumentServices.getOrDefault(fileExtension, null);
    }

    @Override
    public CompletableFuture<WorkspaceEdit> rename(RenameParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.rename(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public void didOpen(DidOpenTextDocumentParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument().getUri());
        if (documentService != null) {
            documentService.didOpen(arg);
        }
    }

    @Override
    public void didChange(DidChangeTextDocumentParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            documentService.didChange(arg);
        }
    }

    @Override
    public void didClose(DidCloseTextDocumentParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            documentService.didClose(arg);
        }
    }

    @Override
    public void didSave(DidSaveTextDocumentParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            documentService.didSave(arg);
        }
    }

    @Override
    public void willSave(WillSaveTextDocumentParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            documentService.willSave(arg);
        }
    }

    @Override
    public CompletableFuture<Either<List<CompletionItem>, CompletionList>> completion(CompletionParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.completion(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<Hover> hover(TextDocumentPositionParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.hover(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<SignatureHelp> signatureHelp(TextDocumentPositionParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.signatureHelp(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<Either<List<? extends Location>, List<? extends LocationLink>>> declaration(TextDocumentPositionParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.declaration(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<Either<List<? extends Location>, List<? extends LocationLink>>> definition(TextDocumentPositionParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.definition(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<Either<List<? extends Location>, List<? extends LocationLink>>> typeDefinition(TextDocumentPositionParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.typeDefinition(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<Either<List<? extends Location>, List<? extends LocationLink>>> implementation(TextDocumentPositionParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.implementation(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<? extends Location>> references(ReferenceParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.references(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<? extends DocumentHighlight>> documentHighlight(TextDocumentPositionParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.documentHighlight(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<Either<SymbolInformation, DocumentSymbol>>> documentSymbol(DocumentSymbolParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.documentSymbol(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<Either<Command, CodeAction>>> codeAction(CodeActionParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.codeAction(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<? extends CodeLens>> codeLens(CodeLensParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.codeLens(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }


    @Override
    public CompletableFuture<List<? extends TextEdit>> formatting(DocumentFormattingParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.formatting(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<? extends TextEdit>> rangeFormatting(DocumentRangeFormattingParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.rangeFormatting(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<? extends TextEdit>> onTypeFormatting(DocumentOnTypeFormattingParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.onTypeFormatting(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<TextEdit>> willSaveWaitUntil(WillSaveTextDocumentParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.willSaveWaitUntil(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<DocumentLink>> documentLink(DocumentLinkParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.documentLink(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<ColorInformation>> documentColor(DocumentColorParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.documentColor(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<ColorPresentation>> colorPresentation(ColorPresentationParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.colorPresentation(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<List<FoldingRange>> foldingRange(FoldingRangeRequestParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.foldingRange(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<Either<Range, PrepareRenameResult>> prepareRename(TextDocumentPositionParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.prepareRename(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<TypeHierarchyItem> typeHierarchy(TypeHierarchyParams arg) {

        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.typeHierarchy(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
    }

    @Override
    public CompletableFuture<CallHierarchyItem> callHierarchy(CallHierarchyParams arg) {
        TextDocumentService documentService = getMatchingDocumentService(arg.getTextDocument());
        if (documentService != null) {
            try {
                return documentService.callHierarchy(arg);
            } catch (UnsupportedOperationException e) {
            }
        }
        return CompletableFuture.completedFuture(null);
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
