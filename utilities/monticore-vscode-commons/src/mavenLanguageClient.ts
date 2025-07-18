import * as vscode from 'vscode';
import { ChildProcess } from 'child_process';
import { LanguageClient, ServerOptions, MessageTransports, LanguageClientOptions } from 'vscode-languageclient';
import { join } from 'path';
import { SocketMessageReader, SocketMessageWriter } from 'vscode-jsonrpc';
import { Socket } from 'net';
import { getLogger, Logger } from 'log4js';
import { ChunkBuffer } from './chunkBuffer';
import { spawnMavenExecChildProcess, spawnDockerMavenExecChildProcess } from './utils';

export interface MavenLanguageClientOptions {
    languages: string[];
    useRunningServer: boolean;
    manualPort: number;
    bufferSize: number;
    pomRoot: string;
    relativeMvnSettingsPath: string | null;
    useDocker?: boolean;
    dockerImage? :string;
}

export class MavenLanguageClient {
    private client: LanguageClient | null;
    private serverBuffer: ChunkBuffer;
    private constants: MavenLanguageClientOptions;
    private connected: boolean;
    private lspProcess: ChildProcess | null;
    private processRunning: boolean;
    private logger: Logger;
    private context: vscode.ExtensionContext;

    constructor(context: vscode.ExtensionContext, constants: MavenLanguageClientOptions) {
        this.constants = constants;
        this.serverBuffer = new ChunkBuffer(constants.bufferSize);
        this.connected = false;
        this.client = null;
        this.lspProcess = null;
        this.logger = getLogger(this.getLanguagesString());
        this.context = context;
        this.processRunning = false;
    }

    startServerAndConnect(clientOptions: LanguageClientOptions) {
        const mavenPath = join(this.context.extensionPath, this.constants.pomRoot);
        this.logger.trace("maven path: " + mavenPath);

        let statusBarItem = vscode.window.createStatusBarItem();
        statusBarItem.text = this.getLanguagesString() + ": starting Language Server";
        statusBarItem.show();
  
        let docker = this.constants.useDocker;
        if(this.constants.useDocker){
            if(!this.constants.dockerImage){
                this.logger.error("useDocker is true but dockerImage is not set!")
                docker = false;
            }

            if(this.constants.manualPort == 0){
                this.logger.error("useDocker is true but manualPort is set to 0. Please specify a port to use!");
                docker = false;
            }
        }
        
        if(docker){
            const baseDir = vscode.workspace.rootPath;
            this.lspProcess = spawnDockerMavenExecChildProcess(
                this.constants.dockerImage!,
                mavenPath,
                [
                    '-p ' + this.constants.manualPort,
                    "--host", process.platform,
                    "-d",
                    "--from", baseDir!,
                    "--to", "/home/isabelle/lsp/"
                ],
                [
                    "-p", this.constants.manualPort + ":" + this.constants.manualPort,
                    "-v", baseDir + ":/home/isabelle/lsp/"
                ],
                this.constants.relativeMvnSettingsPath ? ["-s", this.constants.relativeMvnSettingsPath] : undefined
            )
        }else{
            this.lspProcess = spawnMavenExecChildProcess(
                mavenPath,
                ['-p ' + this.constants.manualPort],
                this.constants.relativeMvnSettingsPath ? ["-s", this.constants.relativeMvnSettingsPath] : undefined
            )
    }

        this.processRunning = true;

        this.lspProcess.on("exit", (code, signal) => this.logger.error("code: " + code + ", signal: " + signal));
        if(this.lspProcess.stdout !== null){
            this.lspProcess.stdout.on("data", (data) => {
                const dataAsString = "" + data;
                this.serverBuffer.enq(dataAsString);
                let curText = this.serverBuffer.getText();
    
                if(curText.includes("Downloading")){
                    statusBarItem.text = this.getLanguagesString() + ": starting Language Server, Downloading dependencies";
                }
    
                if (!this.connected && (curText).includes("Listening on port ")) {
                    let portRegex = /.*Listening on port (\d+).*?[\r]?\n/;
                    let portMatch = portRegex.exec(curText);
                    if (portMatch) {
                        let portStr = portMatch[1];
                        let port = parseInt(portStr);
                        getLogger(this.getLanguagesString()).debug("Connecting on port " + port);
                        statusBarItem.hide();
    
                        this.connected = true;
                        this.connectToServer(clientOptions, port, this.getLanguagesString() + "LangClient");
                    } else {
                        this.logger.debug("Can not find port!");
                    }
                }
            });
        }else{
            this.logger.error("Stdout of process in null!");
        }
       

        this.lspProcess.on("close", (code, signal) => {
            this.logger.error("The server exited with code " + code + ". The last " + this.constants.bufferSize + " chunks of output follows:");
            this.stop();
        });

        this.lspProcess.on("exit", (code, signal) => this.stop());

        this.lspProcess.on("error", (err) => {
             this.logger.error("Maven process error: " + err);
             this.stop();
        });
    }

    connectToServer(clientOptions: LanguageClientOptions, port: number, languageClientName: string) {
        const socket = new Socket().connect({ port: port });
        let serverOptions: ServerOptions = () => Promise.resolve(messageTransports);
        let messageTransports: MessageTransports = {
            reader: new SocketMessageReader(socket, "utf-8"),
            writer: new SocketMessageWriter(socket, "utf-8")
        };
        this.client = new LanguageClient(languageClientName, serverOptions, clientOptions);
        this.logger.debug("build client");
        this.client.registerProposedFeatures();
        this.logger.debug("features");
        this.logger.info("Connected!");
        this.context.subscriptions.push(this.client.start());
    }

    connect() {
        let selectors = [];
        for(let lang of this.constants.languages){
            selectors.push(
                {
                    scheme: 'file',
                    language: lang
                }
            );
        }

        let clientOptions: LanguageClientOptions = {
            documentSelector: selectors
        };

        if (!this.constants.useRunningServer) {
            this.startServerAndConnect(clientOptions);
        } else {
            this.connectToServer(clientOptions, this.constants.manualPort, this.getLanguagesString() + "LangClient");
        }
    }

    stop() {
        let result = "\n" + this.serverBuffer.getText();
        this.logger.error(result);

        if (this.lspProcess) {
            this.lspProcess.kill();
            this.lspProcess = null;
        }

        if (this.client) {
            this.client.stop();
            this.client = null;
        }

        this.connected = false;
        this.processRunning = false;
        this.serverBuffer = new ChunkBuffer(this.constants.bufferSize);
    }

    isConnected(): boolean {
        return this.connected;
    }

    isProcessActive(): boolean {
        if(!this.lspProcess){
            return false;
        }
        return this.processRunning;
    }

    getLanguageIds(): string[] {
        return this.constants.languages;
    }

    getLanguagesString(): string{
        return this.constants.languages.join("|");
    }

}