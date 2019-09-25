/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CancellationToken } from "@theia/core/lib/common/cancellation";
import { injectable } from "inversify";
import { Message, MessageClient, ProgressMessage, ProgressUpdate } from "@theia/core/lib/common/message-service-protocol";

import * as EventEmitter from "eventemitter3";

import ListenerFn = EventEmitter.ListenerFn;

@injectable()
export class NotificationsClientImpl extends MessageClient {
    protected readonly events: EventEmitter;

    protected message: Message | undefined;
    protected update: ProgressUpdate | undefined;

    public constructor() {
        super();

        this.events = new EventEmitter();
    }

    public on(event: string, handler: ListenerFn): void {
        this.events.on(event, handler);
    }

    public off(event: string, handler: ListenerFn): void {
        this.events.off(event, handler);
    }

    // tslint:disable-next-line:no-any
    protected emit(event: string, ...args: any[]): void {
        this.events.emit(event, ...args);
    }

    public getCurrentMessage(): Message | undefined {
        return this.message;
    }

    public getCurrentUpdate(): ProgressUpdate | undefined {
        return this.update;
    }

    public async showMessage(message: Message): Promise<string | undefined> {
        this.message = message;

        this.emit("show-message", message);
        return undefined;
    }

    public async showProgress(progressId: string, message: ProgressMessage,
                              cancellationToken: CancellationToken): Promise<string | undefined> {
        this.message = message;

        this.emit("show-progress", progressId, message, cancellationToken);
        return undefined;
    }

    public async reportProgress(progressId: string, update: ProgressUpdate, message: ProgressMessage,
                                cancellationToken: CancellationToken): Promise<void> {
        this.message = message;
        this.update = update;

        this.emit("report-progress", progressId, update, message, cancellationToken);
    }
}
