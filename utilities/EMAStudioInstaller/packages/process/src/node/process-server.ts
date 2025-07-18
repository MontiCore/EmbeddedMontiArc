/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable } from "inversify";
import { ProcessDescription, ProcessOptions, ProcessServer, ProcessType } from "../common/process-protocol";
import { RawProcessFactory, TerminalProcessFactory, TerminalProcessOptions } from "@theia/process/lib/node";
import { RawProcessOptions } from "@theia/process/lib/node/raw-process";

@injectable()
export class ProcessServerImpl implements ProcessServer {
    @inject(RawProcessFactory) protected readonly rawProcessFactory: RawProcessFactory;
    @inject(TerminalProcessFactory) protected readonly terminalProcessFactory: TerminalProcessFactory;

    public async spawn(options: ProcessDescription): Promise<number | undefined> {
        if (options.type === ProcessType.Raw) return this.spawnRaw(options.options);
        else if (options.type === ProcessType.Terminal) return this.spawnTerminal(options.options);
        else return undefined;
    }

    protected async spawnRaw(options: ProcessOptions): Promise<number> {
        const rawOptions = <RawProcessOptions>options;
        const rawProcess = this.rawProcessFactory(rawOptions);

        return rawProcess.id;
    }

    protected async spawnTerminal(options: ProcessOptions): Promise<number> {
        const terminalOptions = <TerminalProcessOptions>options;
        const terminalProcess = this.terminalProcessFactory(terminalOptions);

        return terminalProcess.id;
    }
}
