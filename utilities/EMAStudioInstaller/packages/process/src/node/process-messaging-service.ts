/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import {
    IProcessExitEvent, ProcessManager,
    RawProcess, TerminalProcess
} from "@theia/process/lib/node";
import { MessagingService } from "@theia/core/lib/node";
import { PROCESSES_PATH } from "../common";
import { Readable } from "stream";
import { MessageConnection } from "vscode-jsonrpc";

import MessagingServiceContribution = MessagingService.Contribution;

@injectable()
export class ProcessMessagingService implements MessagingServiceContribution {
    @inject(ProcessManager) protected readonly processManager: ProcessManager;

    public configure(service: MessagingService): void {
        service.listen(`${PROCESSES_PATH}/:id`, this.onServiceListen.bind(this));
    }

    protected onServiceListen(params: { id: string }, connection: MessageConnection): void {
        const id = parseInt(params.id, 10);
        const process = this.processManager.get(id);

        if (process instanceof RawProcess) this.bindRawProcess(process, connection);
        else if (process instanceof TerminalProcess) this.bindTerminalProcess(process, connection);
    }

    protected bindRawProcess(process: RawProcess, connection: MessageConnection): void {
        const output = process.output;
        const errorOutput = process.errorOutput;

        this.bindPID(process, connection);
        this.bindKill(process, connection);
        this.bindProcess(process, connection);
        this.bindReadableStream(output, connection, "stdout:data");
        this.bindReadableStream(errorOutput, connection, "stderr:data");

        connection.listen();
    }

    protected bindTerminalProcess(process: TerminalProcess, connection: MessageConnection): void {
        const output = process.createOutputStream();

        this.bindProcess(process, connection);
        this.bindReadableStream(output, connection, "stdout:data");

        connection.onClose(() => output.dispose());
        connection.listen();
    }

    protected bindProcess(process: RawProcess | TerminalProcess, connection: MessageConnection): void {
        this.bindExit(process, connection);
        this.bindError(process, connection);
    }

    protected bindExit(process: RawProcess | TerminalProcess, connection: MessageConnection): void {
        process.onExit((event: IProcessExitEvent) => {
            connection.sendNotification("exit", event);
        });
    }

    protected bindError(process: RawProcess | TerminalProcess, connection: MessageConnection): void {
        process.onError((error: Error) => {
            connection.sendNotification("exit", error.message);
        });
    }

    protected bindReadableStream(stream: Readable, connection: MessageConnection, label: string): void {
        const callback = this.createReadableStreamCallback(connection, stream, label);

        stream.on("data", callback);
        connection.onNotification(`${label}:resume`, () => stream.resume());
        connection.onClose(() => stream.removeListener("data", callback));
    }

    // tslint:disable-next-line:no-any
    protected createReadableStreamCallback(connection: MessageConnection, stream: Readable, label: string): (buffer: any) => void {
        return buffer => {
            const data = buffer.toString();

            stream.pause();
            connection.sendNotification(label, data);
        };
    }

    protected bindKill(process: RawProcess, connection: MessageConnection): void {
        connection.onNotification("kill", signal => {
            process.kill(signal);
        });
    }

    protected bindPID(process: RawProcess, connection: MessageConnection): void {
        connection.onNotification("pid", () => {
            connection.sendNotification("pid", process.pid);
        });
    }
}
