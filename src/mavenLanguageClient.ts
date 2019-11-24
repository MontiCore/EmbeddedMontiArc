import * as vscode from 'vscode';
import { spawn, SpawnOptions, ChildProcess } from 'child_process';
import { LanguageClient, ServerOptions, MessageTransports, LanguageClientOptions } from 'vscode-languageclient';
import { join } from 'path';
import { SocketMessageReader, SocketMessageWriter } from 'vscode-jsonrpc';
import { Socket } from 'net';
import { getLogger, Logger } from 'log4js';
import { ChunkBuffer } from './chunkBuffer';

export interface MavenLanguageClientOptions {
    languageName: string;
    useRunningServer: boolean;
    manualPort: number;
    bufferSize: number;
    pomRoot: string;
    relativeMvnSettingsPath: string | null;
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
        this.logger = getLogger(this.constants.languageName);
        this.context = context;
        this.processRunning = false;
    }

    startServerAndConnect(clientOptions: LanguageClientOptions) {
        const mavenPath = join(this.context.extensionPath, this.constants.pomRoot);
        this.logger.trace("maven path: " + mavenPath);
        let spawnOptions: SpawnOptions = {
            cwd: mavenPath,
            env: process.env,
            stdio: "pipe",
            shell: true
        };
        const args: string[] = ["exec:java", '-e', '-Dexec.args="-p ' + this.constants.manualPort + '"'];
        if (this.constants.relativeMvnSettingsPath) {
            args.push("-s", this.constants.relativeMvnSettingsPath);
        }

        this.logger.trace("spwan(mvn, [" + args + "] , " + spawnOptions.cwd + ")");
        let statusBarItem = vscode.window.createStatusBarItem();
        statusBarItem.text = this.constants.languageName + ": starting Language Server";
        statusBarItem.show();

        this.lspProcess = spawn("mvn", args, spawnOptions);
        this.processRunning = true;

        this.lspProcess.on("exit", (code, signal) => this.logger.error("code: " + code + ", signal: " + signal));
        if(this.lspProcess.stdout !== null){
            this.lspProcess.stdout.on("data", (data) => {
                const dataAsString = "" + data;
                this.serverBuffer.enq(dataAsString);
                let curText = this.serverBuffer.getText();
    
                if(curText.includes("Downloading")){
                    statusBarItem.text = this.constants.languageName + ": starting Language Server, Downloading dependencies";
                }
    
                if (!this.connected && (curText).includes("Listening on port ")) {
                    let portRegex = /.*Listening on port (\d+).*?[\r]?\n/;
                    let portMatch = portRegex.exec(curText);
                    if (portMatch) {
                        let portStr = portMatch[1];
                        let port = parseInt(portStr);
                        getLogger(this.constants.languageName).debug("Connecting on port " + port);
                        statusBarItem.hide();
    
                        this.connected = true;
                        this.connectToServer(clientOptions, port, this.constants.languageName + "LangClient");
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
        let clientOptions: LanguageClientOptions = {
            documentSelector: [{ scheme: 'file', language: this.constants.languageName }],
        };

        if (!this.constants.useRunningServer) {
            this.startServerAndConnect(clientOptions);
        } else {
            this.connectToServer(clientOptions, this.constants.manualPort, this.constants.languageName + "LangClient");
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

    getLanguageId(): string {
        return this.constants.languageName;
    }

}