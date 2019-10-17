import * as vscode from 'vscode';
import { spawn, SpawnOptions, ChildProcess } from 'child_process';
import { LanguageClient, ServerOptions, MessageTransports, LanguageClientOptions } from 'vscode-languageclient';
import { join } from 'path';
import { SocketMessageReader, SocketMessageWriter } from 'vscode-jsonrpc';
import { Socket } from 'net';
import { getLogger, Logger } from 'log4js';
import CircularBuffer from 'circularbuffer';

export interface MavenLanguageClientOptions {
    languageName: string;
    useRunningServer: boolean;
    manualPort: number;
    bufferSize: number;
    pomRoot: string;
    mvnSettingsFile: string | null;
}

export class MavenLanguageClient {
    private client: LanguageClient | null;
    private serverBuffer: CircularBuffer<string>;
    private constants: MavenLanguageClientOptions;
    private connected: boolean;
    private lspProcess: ChildProcess | null;
    private logger: Logger;
    private context: vscode.ExtensionContext;

    constructor(context: vscode.ExtensionContext, constants: MavenLanguageClientOptions) {
        this.constants = constants;
        this.serverBuffer = new CircularBuffer<string>(constants.bufferSize);
        this.connected = false;
        this.client = null;
        this.lspProcess = null;
        this.logger = getLogger(this.constants.languageName);
        this.context = context;
    }

    startServerAndConnect(clientOptions: LanguageClientOptions) {
        const mavenPath = join(this.context.extensionPath, this.constants.pomRoot);
        let spawnOptions: SpawnOptions = {
            cwd: mavenPath,
            env: process.env,
            stdio: "pipe"
        };
        const args: string[] = ["exec:java", '-e', '-Dexec.args="-p ' + this.constants.manualPort + '"'];
        if(this.constants.mvnSettingsFile){
            args.push("-s", join(this.context.extensionPath, this.constants.mvnSettingsFile));
        }

        this.lspProcess = spawn("mvn", args, spawnOptions);

        this.lspProcess.stdout.on("data", (data) => {
            const dataAsString = "" + data;
            this.serverBuffer.enq(dataAsString);
            let curText = this.getText(this.serverBuffer);
            if (!this.connected && (curText).includes("Listening on port ")) {
                let portRegex = /.*Listening on port (\d+).*?\n/;
                let portMatch = portRegex.exec(curText);
                if (portMatch) {
                    let portStr = portMatch[1];
                    let port = parseInt(portStr);
                    getLogger(this.constants.languageName).debug("Connecting on port " + port);
                    this.connected = true;
                    this.connectToServer(clientOptions, port, this.constants.languageName + "LangClient");
                } else {
                    this.logger.debug("Can not find port!");
                }
            }
        });

        this.lspProcess.on("close", (code, signal) => {
            this.logger.error("The server exited with code " + code + ". The last " + this.constants.bufferSize + " chunks of output follows:");
            let result = "\n" + this.getText(this.serverBuffer);
            this.logger.error(result);
            this.stop();
        });
    }

    getText(buffer: CircularBuffer<string>): string {
        let res = "";
        for (let i of buffer.toArray()) {
            res += i;
        }
        return res;
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
        if (this.lspProcess) {
            this.lspProcess.kill();
        }

        if (this.client) {
            this.client.stop();
        }

        this.connected = false;
    }

    isConnected(): boolean {
        return this.connected;
    }

    getLanguageId(): string {
        return this.constants.languageName;
    }

}