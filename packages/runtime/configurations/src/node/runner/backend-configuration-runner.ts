/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CancellationToken } from "@theia/core/lib/common";
import { IProcessExitEvent, Process } from "@theia/process/lib/node";
import { injectable } from "inversify";
import { CommonConfigurationRunner } from "../../common";

@injectable()
export abstract class CommonBackendConfigurationRunner<V> extends CommonConfigurationRunner<V> {
    protected waitForOutput(process: Process, search: string, token: CancellationToken): Promise<void> {
        return new Promise((resolve, reject) => {
            let buffer = '';
            const handler = (data: string) => {
                buffer += data;

                if (buffer.indexOf(search) > -1) {
                    process.errorStream.off("data", handler);
                    process.outputStream.off("data", handler);
                    resolve();
                }
            };

            process.errorStream.on("data", handler);
            process.outputStream.on("data", handler);
            token.onCancellationRequested(resolve);
        });
    }

    protected waitForTermination(process: Process, token: CancellationToken): Promise<IProcessExitEvent> {
        return new Promise<IProcessExitEvent>(resolve => {
            process.onExit(resolve);
            token.onCancellationRequested(() => resolve({ code: 1, signal: "SIGTERM" }));
        });
    }
}
