/*
 * (c) https://github.com/MontiCore/monticore
 */
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { bind } from "helpful-decorators";
import { inject, injectable } from "inversify";
import { Disposable, DisposableCollection, Emitter, Event } from "@theia/core/lib/common";
import { ConfigurationManager } from "../configuration-manager";

import {
    Configuration,
    ConfigurationOutputEvent,
    ConfigurationProcessorWatcher,
    ConfigurationStartEvent
} from "../../common";

export interface BufferClearEvent {
    readonly uuid: string;
}

export interface BufferRemoveEvent {
    readonly uuid: string;
}

export interface BufferChangeEvent {
    readonly uuid: string;
    readonly data: string;
}

export const ConfigurationProcessorBuffer = Symbol("ConfigurationProcessorBuffer");
export interface ConfigurationProcessorBuffer {
    readonly onChanged: Event<BufferChangeEvent>;
    readonly onRemoved: Event<BufferRemoveEvent>;
    readonly onCleared: Event<BufferClearEvent>;

    get(uuid: string): string | undefined;
    clear(uuid: string): void;
    remove(uuid: string): void;
    getBufferedConfigurations(): Promise<Configuration[]>;
}

@injectable()
export class ConfigurationProcessorBufferImpl implements ConfigurationProcessorBuffer, FrontendApplicationContribution, Disposable {
    @inject(ConfigurationProcessorWatcher) protected readonly watcher: ConfigurationProcessorWatcher;
    @inject(ConfigurationManager) protected readonly manager: ConfigurationManager;

    protected readonly buffers: Map<string, string>;
    protected readonly toDispose: DisposableCollection;

    protected readonly onClearEmitter: Emitter<BufferClearEvent>;
    protected readonly onRemoveEmitter: Emitter<BufferRemoveEvent>;
    protected readonly onChangeEmitter: Emitter<BufferChangeEvent>;

    public constructor() {
        this.buffers = new Map();
        this.toDispose = new DisposableCollection();

        this.onClearEmitter = new Emitter();
        this.onRemoveEmitter = new Emitter();
        this.onChangeEmitter = new Emitter();
    }

    public get(uuid: string): string | undefined {
        return this.buffers.get(uuid);
    }

    public clear(uuid: string): void {
        this.buffers.set(uuid, "");
        this.fireClear(uuid);
    }

    public remove(uuid: string): void {
        this.buffers.delete(uuid);
        this.fireRemove(uuid);
    }

    public async getBufferedConfigurations(): Promise<Configuration[]> {
        const configurations = await this.manager.getConfigurations();
        const uuids = [...this.buffers.keys()];

        return configurations.filter(configuration => uuids.indexOf(configuration.uuid) > -1);
    }

    protected normalizeData(data: string): string {
        return data.replace(/(?<!\r)\n/gm, "\r\n");
    }

    protected fireChange(event: ConfigurationOutputEvent): void {
        this.onChangeEmitter.fire(event);
    }

    protected fireClear(uuid: string): void {
        this.onClearEmitter.fire({ uuid });
    }

    protected fireRemove(uuid: string): void {
        this.onRemoveEmitter.fire({ uuid });
    }

    public dispose(): void {
        this.toDispose.dispose();
    }

    public get onChanged(): Event<BufferChangeEvent> {
        return this.onChangeEmitter.event;
    }

    public get onCleared(): Event<BufferClearEvent> {
        return this.onClearEmitter.event;
    }

    public get onRemoved(): Event<BufferRemoveEvent> {
        return this.onRemoveEmitter.event;
    }

    public onStart(): void {
        this.toDispose.pushAll([
            this.onClearEmitter,
            this.onRemoveEmitter,
            this.onChangeEmitter,
            Disposable.create(() => this.buffers.clear()),
            this.watcher.onConfigurationOutput(this.onConfigurationOutput),
            this.watcher.onConfigurationStarted(this.onConfigurationStarted)
        ]);
    }

    @bind
    protected onConfigurationOutput(event: ConfigurationOutputEvent): void { // TODO: Limit size.
        const buffer = this.buffers.get(event.uuid);
        const normalizedEvent = { uuid: event.uuid, data: this.normalizeData(event.data) };

        if (buffer) this.buffers.set(event.uuid, buffer + normalizedEvent.data);
        else this.buffers.set(event.uuid, normalizedEvent.data);

        this.fireChange(normalizedEvent);
    }

    @bind
    protected onConfigurationStarted(event: ConfigurationStartEvent): void {
        this.clear(event.uuid);
    }
}
