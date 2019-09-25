/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.document;

import com.google.common.flogger.FluentLogger;
import com.google.inject.Singleton;
import org.eclipse.lsp4j.DidChangeTextDocumentParams;
import org.eclipse.lsp4j.DidCloseTextDocumentParams;
import org.eclipse.lsp4j.DidOpenTextDocumentParams;
import org.eclipse.lsp4j.TextDocumentItem;

import java.util.HashMap;
import java.util.Map;

@Singleton
public class TextDocuments {
    protected final FluentLogger logger;
    protected final Map<String, TextDocument> documents;

    protected TextDocuments() {
        this.logger = FluentLogger.forEnclosingClass();
        this.documents = new HashMap<>();
    }

    public TextDocument get(String uri) {
        return this.documents.get(uri);
    }

    public void onDidOpenTextDocumentParams(DidOpenTextDocumentParams params) {
        TextDocumentItem document = params.getTextDocument();

        this.documents.put(document.getUri(), new TextDocument(document));
    }

    public void onDidChangeTextDocumentParams(DidChangeTextDocumentParams params) {
        TextDocument document = this.get(params.getTextDocument().getUri());

        if (document != null) document.update(params.getContentChanges());
    }

    public void onDidCloseTextDocumentParams(DidCloseTextDocumentParams params) {
        this.documents.remove(params.getTextDocument().getUri());
    }
}
