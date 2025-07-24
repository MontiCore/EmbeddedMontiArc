/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationTokenSource, Event, Emitter } from "@theia/core/lib/common";
import { inject, injectable } from "inversify";
import {
    Configuration,
    ConfigurationExitEvent,
    ConfigurationOutputEvent,
    ConfigurationRunnerClient,
    ConfigurationStartEvent
} from "../../common";
import { ConfigurationCoordinatorDelegator } from "../coordinator";
import { Process } from "@theia/process/lib/node";

export const ConfigurationProcessor = Symbol("ConfigurationProcessor");
export interface ConfigurationProcessor {
    readonly onConfigurationStart: Event<ConfigurationStartEvent>;
    readonly onConfigurationExit: Event<ConfigurationExitEvent>;
    readonly onConfigurationOutput: Event<ConfigurationOutputEvent>;

    isRunning(uuid: string): Promise<boolean>;
    getRunningConfigurations(): Promise<string[]>;
    run(configuration: Configuration, context: OptionsContext): Promise<void>;
    kill(uuid: string): Promise<void>;
    registerProcess(uuid: string, process: Process): void;
}

@injectable()
export class ConfigurationProcessorImpl implements ConfigurationProcessor {
    @inject(ConfigurationCoordinatorDelegator) protected readonly coordinator: ConfigurationCoordinatorDelegator;
    @inject(ConfigurationRunnerClient) protected readonly client: ConfigurationRunnerClient;

    protected readonly tokens: Map<string, CancellationTokenSource>;
    protected readonly processes: Map<string, Process[]>;

    protected readonly onConfigurationExitEmitter: Emitter<ConfigurationExitEvent>;
    protected readonly onConfigurationStartEmitter: Emitter<ConfigurationStartEvent>;
    protected readonly onConfigurationOutputEmitter: Emitter<ConfigurationOutputEvent>;

    protected ids: number;

    public constructor() {
        this.tokens = new Map();
        this.processes = new Map();
        this.ids = 0;

        this.onConfigurationExitEmitter = new Emitter();
        this.onConfigurationStartEmitter = new Emitter();
        this.onConfigurationOutputEmitter = new Emitter();
    }

    public async isRunning(uuid: string): Promise<boolean> {
        return this.tokens.has(uuid);
    }

    public async getRunningConfigurations(): Promise<string[]> {
        return [...this.tokens.keys()];
    }

    public async run(configuration: Configuration, context: OptionsContext): Promise<void> {
        const source = new CancellationTokenSource();

        this.tokens.set(configuration.uuid, source);
        this.fireConfigurationStart(configuration);

        try {
            await this.coordinator.run(configuration, context, source.token);
            await this.waitForProcessesTermination(configuration.uuid);
        } catch (error) {
            source.cancel();
            await this.client.kill(configuration.uuid);
            this.fireConfigurationOutput(configuration.uuid, error.message);
            this.fireConfigurationExit(configuration);
        }

        source.dispose();
        await this.client.dispose(configuration.uuid);
        this.tokens.delete(configuration.uuid);
        this.fireConfigurationExit(configuration);
    }

    public async kill(uuid: string): Promise<void> {
        const token = this.tokens.get(uuid);

        if (token) token.cancel();
    }

    public registerProcess(uuid: string, process: Process): void {
        const processes = this.processes.get(uuid);

        if (processes) processes.push(process);
        else this.processes.set(uuid, [process]);

        this.bindProcessEventHandlers(uuid, process);
    }

    protected unregisterProcess(uuid: string, process: Process): void {
        const processes = this.processes.get(uuid);
        const index = processes && processes.indexOf(process);

        if (processes && index && index > -1) processes.splice(index, 1);

        process.errorStream.removeAllListeners("data");
        process.outputStream.removeAllListeners("data");
    }

    protected async waitForProcessesTermination(uuid: string): Promise<void> {
        const processes = this.processes.get(uuid);

        if (processes) return this.doWaitForProcessesTermination(processes);
    }

    protected async doWaitForProcessesTermination(processes: Process[]): Promise<void> {
        const mapping = (process: Process) => new Promise(resolve => {
            if (process.killed) {
                resolve();
            } else {
                process.onExit(resolve);
                process.onError(resolve);
            }
        });

        await Promise.all(processes.map(mapping));
    }

    protected fireConfigurationStart(configuration: Configuration): void {
        this.onConfigurationStartEmitter.fire({ uuid: configuration.uuid });
    }

    protected fireConfigurationExit(configuration: Configuration): void {
        this.onConfigurationExitEmitter.fire({ uuid: configuration.uuid });
    }

    protected fireConfigurationOutput(uuid: string, data: string): void {
        this.onConfigurationOutputEmitter.fire({ uuid, data });
    }

    protected bindProcessEventHandlers(uuid: string, process: Process): void {
        const source = this.tokens.get(uuid);
        const handler = (data: string) => this.fireConfigurationOutput(uuid, data.toString());

        process.errorStream.on("data", handler);
        process.outputStream.on("data", handler);
        process.onError(() => this.unregisterProcess(uuid, process));
        process.onExit(() => this.unregisterProcess(uuid, process));

        if (source) source.token.onCancellationRequested(() => process.kill());
    }

    public get onConfigurationStart(): Event<ConfigurationStartEvent> {
        return this.onConfigurationStartEmitter.event;
    }

    public get onConfigurationExit(): Event<ConfigurationExitEvent> {
        return this.onConfigurationExitEmitter.event;
    }

    public get onConfigurationOutput(): Event<ConfigurationOutputEvent> {
        return this.onConfigurationOutputEmitter.event;
    }
}
