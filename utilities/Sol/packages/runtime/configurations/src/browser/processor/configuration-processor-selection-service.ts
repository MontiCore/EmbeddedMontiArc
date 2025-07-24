/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Disposable, DisposableCollection, Emitter, Event } from "@theia/core/lib/common";
import { bind } from "helpful-decorators";
import { inject, injectable, postConstruct } from "inversify";
import { Configuration, ConfigurationProcessorWatcher, ConfigurationStartEvent } from "../../common";
import { ConfigurationManager } from "../configuration-manager";
import { BufferRemoveEvent, ConfigurationProcessorBuffer } from "./configuration-processor-buffer";

export const ConfigurationProcessorSelectionService = Symbol("ConfigurationProcessorSelectionService");
export interface ConfigurationProcessorSelectionService {
    readonly onSelectionChanged: Event<Configuration | undefined>;

    select(configuration: Configuration | undefined): void;
    getSelection(): Configuration | undefined;
}

@injectable()
export class ConfigurationProcessorSelectionServiceImpl implements ConfigurationProcessorSelectionService, Disposable {
    @inject(ConfigurationProcessorBuffer) protected readonly buffer: ConfigurationProcessorBuffer;
    @inject(ConfigurationProcessorWatcher) protected readonly watcher: ConfigurationProcessorWatcher;
    @inject(ConfigurationManager) protected readonly manager: ConfigurationManager;

    protected readonly toDispose: DisposableCollection;

    protected readonly onSelectionChangeEmitter: Emitter<Configuration | undefined>;

    protected selection: Configuration | undefined;

    public constructor() {
        this.toDispose = new DisposableCollection();

        this.onSelectionChangeEmitter = new Emitter();
    }

    @postConstruct()
    protected init(): void {
        this.toDispose.pushAll([
            this.onSelectionChangeEmitter,
            this.buffer.onRemoved(this.onBufferRemoved),
            this.watcher.onConfigurationStarted(this.onConfigurationStarted)
        ]);
    }

    public select(selection: Configuration | undefined): void {
        this.selection = selection;

        this.onSelectionChangeEmitter.fire(selection);
    }

    public getSelection(): Configuration | undefined {
        return this.selection;
    }

    public get onSelectionChanged(): Event<Configuration | undefined> {
        return this.onSelectionChangeEmitter.event;
    }

    public dispose(): void {
        this.toDispose.dispose();
    }

    @bind
    protected onBufferRemoved(event: BufferRemoveEvent): void {
        if (this.selection && this.selection.uuid === event.uuid) this.select(undefined);
    }

    @bind
    protected async onConfigurationStarted(event: ConfigurationStartEvent): Promise<void> {
        const configuration = await this.manager.getConfiguration(event.uuid);

        if (configuration) this.select(configuration);
    }
}
