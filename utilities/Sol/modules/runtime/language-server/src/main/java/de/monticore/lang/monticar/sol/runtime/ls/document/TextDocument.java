/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.document;

import org.eclipse.lsp4j.TextDocumentContentChangeEvent;
import org.eclipse.lsp4j.TextDocumentItem;

import java.util.List;

/*
 * TODO: Incremental content changes.
 */
public class TextDocument extends TextDocumentItem {
    public TextDocument(TextDocumentItem document) {
        this(document.getUri(), document.getText());

        super.setVersion(document.getVersion());
        super.setLanguageId(document.getLanguageId());
    }

    public TextDocument(String uri, String text) {
        super.setUri(uri);
        super.setText(text);
    }

    public void update(List<TextDocumentContentChangeEvent> changes) {
        TextDocumentContentChangeEvent changeEvent = changes.get(changes.size() - 1);

        super.setText(changeEvent.getText());
    }
}
