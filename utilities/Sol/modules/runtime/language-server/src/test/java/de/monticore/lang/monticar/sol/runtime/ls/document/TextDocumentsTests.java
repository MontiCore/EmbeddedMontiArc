/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.document;

import org.eclipse.lsp4j.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class TextDocumentsTests {
    TextDocumentItem documentItem = new TextDocumentItem("file:///user/some/file.txt", "monticore", 0, "");
    TextDocuments documents = new TextDocuments();

    @BeforeEach
    void openTextDocument() {
        DidOpenTextDocumentParams params = new DidOpenTextDocumentParams(documentItem);

        documents.onDidOpenTextDocumentParams(params);
    }

    @Test
    void testOnDidOpenTextDocumentParams() {
        assertEquals(
            1,
            documents.documents.size(),
            "TextDocuments should have exactly 1 TextDocument."
        );
    }

    @Test
    void testOnDidChangeTextDocumentParams() {
        VersionedTextDocumentIdentifier identifier =
                new VersionedTextDocumentIdentifier(documentItem.getUri(), documentItem.getVersion());
        List<TextDocumentContentChangeEvent> changes =
                Collections.singletonList(new TextDocumentContentChangeEvent("Hello World!"));
        DidChangeTextDocumentParams params = new DidChangeTextDocumentParams(identifier, changes);

        documents.onDidChangeTextDocumentParams(params);

        assertEquals(
            "Hello World!",
            documents.get(params.getTextDocument().getUri()).getText(),
            "TextDocument should have been updated."
        );
    }

    @Test
    void testOnDidCloseTextDocumentParams() {
        TextDocumentIdentifier identifier = new TextDocumentIdentifier(documentItem.getUri());
        DidCloseTextDocumentParams params = new DidCloseTextDocumentParams(identifier);

        documents.onDidCloseTextDocumentParams(params);

        assertEquals(
            0,
            documents.documents.size(),
            "TextDocument should have exactly 0 TextDocument."
        );
    }
}
