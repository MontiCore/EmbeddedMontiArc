/*
 * (c) https://github.com/MontiCore/monticore
 */
import { MessageClient, MessageType, } from "@theia/core/lib/common/message-service-protocol";
import { CancellationToken } from "@theia/core/lib/common/cancellation";
import { inject, injectable } from "inversify";

export const NotificationsService = Symbol("NotificationsService");
/**
 * An interface to be implemented by classes implementing the functionality to show messages / progress reports to
 * the user.
 */
export interface NotificationsService {
    /**
     * Show a message of a given type and with a given text to the user.
     * @param text The text to be shown to the user.
     * @param type The type of the message to be shown to the user.
     */
    showMessage(text: string, type?: MessageType): Promise<void>;

    /**
     * Show the progress of the current activity to the user.
     * @param done The work done as numerical value.
     * @param total The total work as numerical value.
     * @param text An optional text associated to the progress which should be shown to the user.
     */
    reportProgress(done: number, total: number, text?: string): Promise<void>;
}

@injectable()
export class NotificationsServiceImpl implements NotificationsService {
    @inject(MessageClient) protected client: MessageClient;

    public async showMessage(text: string, type?: MessageType): Promise<void> {
        await this.client.showMessage({ text, type });
    }

    public async reportProgress(done: number, total: number, text?: string): Promise<void> {
        const update = { work: { done, total } };
        const message = { text: text || '' };

        return this.client.reportProgress('', update, message, CancellationToken.None);
    }
}
