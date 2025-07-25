/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { LogLevel } from "@theia/core/lib/common/logger";
import { ILoggerServer, ILoggerClient } from "@theia/core/lib/common/logger-protocol";

@injectable()
export class LoggerBrowser implements ILoggerServer {
    protected client: ILoggerClient | undefined;

    public setClient(client: ILoggerClient | undefined): void {
        this.client = client;
    }

    public dispose(): void {
        // NOOP
    }

    public async setLogLevel(id: string, logLevel: number): Promise<void> {
        // NOOP
    }

    public async getLogLevel(id: string): Promise<number> {
        return LogLevel.DEBUG;
    }

    public async log(id: string, logLevel: number, message: string, params: any[]): Promise<void> {
        // NOOP
    }

    public async child(name: string): Promise<void> {
        // NOOP
    }
}
