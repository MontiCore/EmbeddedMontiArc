/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import de.se_rwth.commons.logging.DiagnosticsLog;
import de.se_rwth.commons.logging.Log;
import org.eclipse.lsp4j.*;
import org.eclipse.lsp4j.jsonrpc.Launcher;
import org.eclipse.lsp4j.launch.LSPLauncher;
import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.lsp4j.services.LanguageClientAware;
import org.eclipse.lsp4j.services.LanguageServer;
import org.eclipse.lsp4j.services.WorkspaceService;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Arrays;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

public abstract class McLspServer implements LanguageServer, LanguageClientAware {

    public void startFromArgs(String[] rawArgs) throws InterruptedException, ExecutionException, IOException {
        String[] args = Arrays.stream(rawArgs).flatMap(arg -> Arrays.stream(arg.split(" "))).toArray(String[]::new);
        boolean failed = false;
        Exception err = new Exception("Invalid Arguments");
        DiagnosticsLog.reuse();
        Log.info("Starting server!", "default");

        if(args.length == 0){
            this.start(System.in, System.out);
        }else if(args.length == 2 && args[0].equals("-p")){
            DiagnosticsLog.setLogToStdout(true);
            try{
                int port = Integer.parseInt(args[1]);
                Log.info("Port " + port, "default");
                this.startOnPort(port);
            }catch (Exception e){
                err = e;
                failed = true;
            }
        }else{
            failed = true;
        }

        if(failed){
            Log.warn("Error starting server. Arguments should be: [] | [-p <port>]\nPassed arguments: " + Arrays.toString(args), err);
            System.exit(1);
        }

    }

    public void startOnPort(int port) throws IOException, InterruptedException, ExecutionException {
        ServerSocket serverSocket = new ServerSocket(port);
        Log.info("Listening on port " + serverSocket.getLocalPort(), "default");
        Socket socket = serverSocket.accept();
        Log.info("Starting server on port " + socket.getPort(),"default");
        start(socket.getInputStream(), socket.getOutputStream());
    }

    public void start(InputStream in, OutputStream out) throws InterruptedException, ExecutionException, IOException {
        Launcher<LanguageClient> l = LSPLauncher.createServerLauncher(this, in, out);
        LanguageClient client = l.getRemoteProxy();
        this.connect(client);
        Future<?> startListening = l.startListening();
    }

    public CompletableFuture<InitializeResult> initialize(InitializeParams initializeParams) {
        Log.info("Initializing", "default");
        InitializeResult res = new InitializeResult();
        ServerCapabilities capabilities = new ServerCapabilities();
        capabilities.setTextDocumentSync(TextDocumentSyncKind.Full);
        addCapabilitiesByHandlers(capabilities);
        res.setCapabilities(capabilities);
        return CompletableFuture.completedFuture(res);
    }

    protected void addCapabilitiesByHandlers(ServerCapabilities capabilities) {
        ClientAwareTextDocumentService documentService = getTextDocumentService();
        //TODO: make compile-time type-save
        if(documentService instanceof MontiCoreDocumentService){
            MontiCoreDocumentService mc = (MontiCoreDocumentService) documentService;
            if(!mc.getCompletionHandler().getDelegates().isEmpty()){
                capabilities.setCompletionProvider(new CompletionOptions());
            }

            if(!mc.getDefinitionHandler().getDelegates().isEmpty()){
                capabilities.setDefinitionProvider(true);
            }
        }
    }

    public CompletableFuture<Object> shutdown() {
        Log.info("Shutting down","default");
        CompletableFuture.runAsync(this::exit);
        return CompletableFuture.completedFuture(null);
    }

    public void exit() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Log.info("Stopping McLspServer", "default");
        System.exit(0);
    }

    public abstract ClientAwareTextDocumentService getTextDocumentService();

    public WorkspaceService getWorkspaceService() {
        return new WorkspaceService() {
            @Override
            public void didChangeConfiguration(DidChangeConfigurationParams didChangeConfigurationParams) {
                Log.info("call didChangeConfiguration", "default");
            }

            @Override
            public void didChangeWatchedFiles(DidChangeWatchedFilesParams didChangeWatchedFilesParams) {
                Log.info("call didChangeWatchedFiles", "default");
            }
        };
    }


    @Override
    public void connect(LanguageClient languageClient) {
        Log.info("Setting language client!", "default");
        this.getTextDocumentService().setClient(languageClient);
    }
}
