/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { JsonRpcServer } from "@theia/core/lib/common";

export const CONFIGURATION_VALIDATOR_TYPE: string = "configuration";

export interface Configuration<V = any> { // tslint:disable-line:no-any
    readonly uuid: string;
    readonly typeId: string;
    name: string;
    options: V;
}

export const CONFIGURATION_PROCESSOR_PATH: string = "/services/configuration-processor";

export interface ConfigurationStartEvent {
    readonly uuid: string;
}

export interface ConfigurationExitEvent {
    readonly uuid: string;
}

export interface ConfigurationOutputEvent<D = any> { // tslint:disable-line:no-any
    readonly uuid: string;
    readonly data: D;
}

export interface ConfigurationProcessorClient {
    onConfigurationStart(event: ConfigurationStartEvent): void;
    onConfigurationExit(event: ConfigurationExitEvent): void;
    onConfigurationOutput(event: ConfigurationOutputEvent): void;
}

export const ConfigurationProcessorServer = Symbol("ConfigurationProcessorServer");
export interface ConfigurationProcessorServer extends JsonRpcServer<ConfigurationProcessorClient> {
    isConfigurationRunning(uuid: string): Promise<boolean>;
    getRunningConfigurations(): Promise<string[]>;
    runConfiguration(configuration: Configuration, context: OptionsContext): Promise<void>;
    killConfiguration(uuid: string): Promise<void>;
    unsetClient(client: ConfigurationProcessorClient): void;
}

export const CONFIGURATION_RUNNER_PATH: string = "/services/configuration-runner";

export const ConfigurationRunnerClient = Symbol("ConfigurationRunnerClient");
export interface ConfigurationRunnerClient {
    run<V>(uuid: string, typeId: string, taskName: string, options: V, context: OptionsContext): Promise<void>;
    kill(uuid: string): Promise<void>;
    dispose(uuid: string): Promise<void>;
}
