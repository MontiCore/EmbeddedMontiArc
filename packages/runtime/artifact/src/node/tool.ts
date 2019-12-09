/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ConfigurationProcessor } from "@embeddedmontiarc/sol-runtime-configurations/lib/node";
import { Process, RawProcessFactory } from "@theia/process/lib/node";
import { inject, injectable, unmanaged } from "inversify";

import vector from "string-argv";

export const Tool = Symbol("Tool");
export interface Tool {
    run(argstring: string, uuid: string): Process;
}

@injectable()
export abstract class CommonTool implements Tool {
    @inject(RawProcessFactory) protected readonly factory: RawProcessFactory;
    @inject(ConfigurationProcessor) protected readonly processor: ConfigurationProcessor;

    protected readonly path: string;
    protected readonly prefix: string;
    protected readonly suffix: string;

    protected constructor(@unmanaged() path: string, @unmanaged() prefix: string, @unmanaged() suffix: string) {
        this.path = path;
        this.prefix = prefix || '';
        this.suffix = suffix || '';
    }

    public run(argstring: string, uuid: string): Process {
        const prefixv = vector(this.prefix);
        const suffixv = vector(this.suffix);
        const argv = vector(argstring);
        const command = prefixv[0];
        const args = [...prefixv.slice(1), this.path, ...suffixv, ...argv];
        const rawProcess = this.factory({ command, args, options: { env: process.env } });

        this.processor.registerProcess(uuid, rawProcess);
        return rawProcess;
    }
}

@injectable()
export abstract class CommonVirtualTool implements Tool {
    @inject(RawProcessFactory) protected readonly factory: RawProcessFactory;
    @inject(ConfigurationProcessor) protected readonly processor: ConfigurationProcessor;

    protected readonly command: string;

    protected constructor(@unmanaged() command: string) {
        this.command = command;
    }

    public run(argstring: string, uuid: string): Process {
        const args = vector(argstring);
        const rawProcess = this.factory({ command: this.command, args, options: { env: process.env } });

        this.processor.registerProcess(uuid, rawProcess);
        return rawProcess;
    }
}
