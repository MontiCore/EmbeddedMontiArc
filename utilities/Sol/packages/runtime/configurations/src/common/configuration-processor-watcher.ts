/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Emitter, Event } from "@theia/core/lib/common";
import { injectable } from "inversify";

import {
    ConfigurationExitEvent,
    ConfigurationOutputEvent,
    ConfigurationProcessorClient, ConfigurationStartEvent
} from "./configuration-protocol";

@injectable()
export class ConfigurationProcessorWatcher {
    protected readonly onConfigurationStartEmitter: Emitter<ConfigurationStartEvent>;
    protected readonly onConfigurationExitEmitter: Emitter<ConfigurationExitEvent>;
    protected readonly onConfigurationOutputEmitter: Emitter<ConfigurationOutputEvent>;

    public constructor() {
        this.onConfigurationStartEmitter = new Emitter();
        this.onConfigurationExitEmitter = new Emitter();
        this.onConfigurationOutputEmitter = new Emitter();
    }

    public get onConfigurationStarted(): Event<ConfigurationStartEvent> {
        return this.onConfigurationStartEmitter.event;
    }

    public get onConfigurationExited(): Event<ConfigurationExitEvent> {
        return this.onConfigurationExitEmitter.event;
    }

    public get onConfigurationOutput(): Event<ConfigurationOutputEvent> {
        return this.onConfigurationOutputEmitter.event;
    }

    public getConfigurationProcessorClient(): ConfigurationProcessorClient {
        const onConfigurationStartEmitter = this.onConfigurationStartEmitter;
        const onConfigurationExitEmitter = this.onConfigurationExitEmitter;
        const onConfigurationOutputEmitter = this.onConfigurationOutputEmitter;

        return {
            onConfigurationStart(event: ConfigurationStartEvent): void {
                onConfigurationStartEmitter.fire(event);
            },
            onConfigurationExit(event: ConfigurationExitEvent): void {
                onConfigurationExitEmitter.fire(event);
            },
            onConfigurationOutput(event: ConfigurationOutputEvent): void {
                onConfigurationOutputEmitter.fire(event);
            }
        };
    }
}
