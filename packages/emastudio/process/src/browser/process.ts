/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { Emitter, Event } from "@theia/core/lib/common";
import { ProcessManager } from "./process-manager";
import { IProcessExitEvent } from "../common";
import { WebSocketConnectionProvider, WebSocketOptions } from "@theia/core/lib/browser";
import { ConnectionHandler } from "@theia/core/lib/common/messaging/handler";
import { PROCESSES_PATH } from "../common";
import { MessageConnection } from "vscode-jsonrpc";
import { Deferred } from "@theia/core/lib/common/promise-util";

export const ProcessIdentifier = Symbol("ProcessIdentifier");
export interface ProcessIdentifier {
    readonly id: number;
    readonly label: string;
}

export const ProcessFactory = Symbol("ProcessFactory");
export interface ProcessFactory {
    (identifier: ProcessIdentifier): Process;
}

@injectable()
export class Process {
    protected _pid: Deferred<number>;
    protected _killed: boolean;

    protected connection: MessageConnection | undefined;

    protected readonly stdoutEmitter: Emitter<string>;
    protected readonly stderrEmitter: Emitter<string>;
    protected readonly exitEmitter: Emitter<IProcessExitEvent>;
    protected readonly errorEmitter: Emitter<Error>;

    protected readonly _id: number;
    protected readonly _label: string;

    protected constructor(
        @inject(ProcessIdentifier) identifier: ProcessIdentifier,
        @inject(ProcessManager) processManager: ProcessManager,
        @inject(WebSocketConnectionProvider) webSocketConnectionProvider: WebSocketConnectionProvider
    ) {
        this._id = identifier.id;
        this._label = identifier.label;
        this._pid = new Deferred<number>();

        this.connection = undefined;
        this.stdoutEmitter = new Emitter<string>();
        this.stderrEmitter = new Emitter<string>();
        this.exitEmitter = new Emitter<IProcessExitEvent>();
        this.errorEmitter = new Emitter<Error>();

        processManager.register(this);
        this.connect(webSocketConnectionProvider);
    }

    protected connect(webSocketConnectionProvider: WebSocketConnectionProvider): void {
        const options = <WebSocketOptions>{ reconnecting: false };
        const handler = <ConnectionHandler>{
            path: `${PROCESSES_PATH}/${this.id}`,
            onConnection: this.onConnection.bind(this)
        };

        webSocketConnectionProvider.listen(handler, options);
    }

    protected onConnection(connection: MessageConnection): void {
        this.connection = connection;

        this.addPIDHandler();
        this.addExitHandler();
        this.addErrorHandler();
        this.addOutputHandler();
        this.addErrorOutputHandler();

        this.connection.listen();
    }

    protected addOutputHandler(): void {
        this.connection!.onNotification("stdout:data", data => {
            this.emitOnOutput(data);

            if (this.connection) this.connection.sendNotification("stdout:data:resume");
        });
    }

    protected addErrorOutputHandler(): void {
        this.connection!.onNotification("stderr:data", data => {
            this.emitOnErrorOutput(data);

            if (this.connection) this.connection.sendNotification("stderr:data:resume");
        });
    }

    protected addPIDHandler(): void {
        this.connection!.onNotification("pid", pid => {
            this._pid.resolve(pid);
        });
    }

    protected addExitHandler(): void {
        this.connection!.onNotification("exit", event => {
            this.emitOnExit(event);
        });
    }

    protected addErrorHandler(): void {
        this.connection!.onNotification("error", message => {
            this.emitOnError(new Error(message));
        });
    }

    public async kill(signal?: string): Promise<void> {
        if (this.connection) this.connection.sendNotification("kill", signal);
    }

    public get pid(): Promise<number> {
        if (this.connection) this.connection.sendNotification("pid");

        return this._pid.promise;
    }

    public get id(): number {
        return this._id;
    }

    public get label(): string {
        return this._label;
    }

    public get killed(): boolean {
        return this._killed;
    }

    public get onOutput(): Event<string> {
        return this.stdoutEmitter.event;
    }

    public get onErrorOutput(): Event<string> {
        return this.stderrEmitter.event;
    }

    public get onExit(): Event<IProcessExitEvent> {
        return this.exitEmitter.event;
    }

    public get onError(): Event<Error> {
        return this.errorEmitter.event;
    }

    protected emitOnExit(event: IProcessExitEvent): void {
        this.handleOnExit();
        this.exitEmitter.fire(event);
    }

    protected handleOnExit(): void {
        // if (this.connection) this.connection.dispose();

        this._killed = true;
        this.connection = undefined;
    }

    protected emitOnOutput(data: string): void {
        this.stdoutEmitter.fire(data);
    }

    protected emitOnErrorOutput(data: string): void {
        this.stderrEmitter.fire(data);
    }

    protected emitOnError(error: Error): void {
        this.handleOnError();
        this.errorEmitter.fire(error);
    }

    protected handleOnError(): void {
        // if (this.connection) this.connection.dispose();

        this._killed = true;
        this.connection = undefined;
    }
}
