package de.monticore.lang.embeddedmontiarc.lsp;

import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.SystemUtils;
import org.eclipse.lsp4j.*;
import org.eclipse.lsp4j.services.LanguageClient;

import java.io.File;
import java.io.IOException;
import java.util.concurrent.CompletableFuture;

public abstract class AbstractTextDocumentServiceTest {

    public LanguageClient getMockClient(){
        return new LanguageClient() {
            @Override
            public void telemetryEvent(Object o) {

            }

            @Override
            public void publishDiagnostics(PublishDiagnosticsParams publishDiagnosticsParams) {

            }

            @Override
            public void showMessage(MessageParams messageParams) {

            }

            @Override
            public CompletableFuture<MessageActionItem> showMessageRequest(ShowMessageRequestParams showMessageRequestParams) {
                return null;
            }

            @Override
            public void logMessage(MessageParams messageParams) {

            }
        };
    }

    public DidOpenTextDocumentParams createDidOpenEvent(File file, String languageId) throws IOException {
        DidOpenTextDocumentParams res = new DidOpenTextDocumentParams();
        res.setTextDocument(createTextDocument(file, languageId));
        return res;
    }

    public TextDocumentItem createTextDocument(File file, String languageId) throws IOException {
        String text = FileUtils.readFileToString(file, "UTF-8");
        String uri = "file://";
        if(SystemUtils.IS_OS_WINDOWS){
           uri += formatWindowsPath(file.getAbsolutePath());
        }else{
            uri += file.getAbsolutePath();
        }
        TextDocumentItem res = new TextDocumentItem(uri, languageId, 1, text);
        return res;
    }

    public String formatWindowsPath(String rawPath){
        String[] parts = rawPath.split("[:]");
        parts[0] = parts[0].toLowerCase();
        String res = String.join(":", parts);
        return res.replace("\\", "/");
    }

}
