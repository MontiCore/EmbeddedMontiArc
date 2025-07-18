/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.document;

import org.eclipse.lsp4j.TextDocumentContentChangeEvent;
import org.eclipse.lsp4j.TextDocumentItem;
import org.junit.jupiter.api.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class TextDocumentTests {
    TextDocumentItem documentItem = new TextDocumentItem("file:///user/some/file.txt", "monticore", 0, "");
    TextDocument document = new TextDocument(documentItem);

    @Test
    void testUpdate() {
        List<TextDocumentContentChangeEvent> changes =
                Collections.singletonList(new TextDocumentContentChangeEvent("Hello World!"));

        document.update(changes);

        assertEquals("Hello World!", document.getText(), "Document has not correctly been updated.");
    }
}
