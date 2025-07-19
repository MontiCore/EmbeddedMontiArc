/*
 * (c) https://github.com/MontiCore/monticore
 */
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { FileSystem } from "@theia/filesystem/lib/common";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { inject, injectable } from "inversify";
import { Configuration } from "../common";
import { Disposable, DisposableCollection, Emitter, Event } from "@theia/core/lib/common";
import { Deferred } from "@theia/core/lib/common/promise-util";

import URI from "@theia/core/lib/common/uri";

export const ConfigurationManager = Symbol("ConfigurationManager");
export interface ConfigurationManager {
    readonly onConfigurationsLoaded: Event<Configuration[]>;
    readonly onConfigurationsAdded: Event<Configuration[]>;
    readonly onConfigurationChanged: Event<Configuration>;
    readonly onConfigurationRemoved: Event<string>;

    setConfiguration<V>(uuid: string, name: string, options: V): Promise<void>;
    addConfiguration(configuration: Configuration): Promise<void>;
    addConfigurations(configurations: Configuration[]): Promise<void>;
    getConfiguration(uuid: string): Promise<Configuration | undefined>;
    removeConfiguration(uuid: string): Promise<void>;
    getConfigurations(): Promise<Configuration[]>;
}

@injectable()
export class ConfigurationManagerImpl implements ConfigurationManager, FrontendApplicationContribution, Disposable {
    @inject(WorkspaceService) protected readonly workspace: WorkspaceService;
    @inject(FileSystem) protected readonly fileSystem: FileSystem;

    protected readonly toDispose: DisposableCollection;
    protected readonly configurations: Map<string, Configuration>;

    protected readonly onConfigurationsLoadEmitter: Emitter<Configuration[]>;
    protected readonly onConfigurationsAddEmitter: Emitter<Configuration[]>;
    protected readonly onConfigurationChangeEmitter: Emitter<Configuration>;
    protected readonly onConfigurationRemoveEmitter: Emitter<string>;

    protected ready: Deferred<void>;

    public constructor() {
        this.toDispose = new DisposableCollection();
        this.configurations = new Map();

        this.onConfigurationsLoadEmitter = new Emitter();
        this.onConfigurationsAddEmitter = new Emitter();
        this.onConfigurationChangeEmitter = new Emitter();
        this.onConfigurationRemoveEmitter = new Emitter();

        this.toDispose.pushAll([
            this.onConfigurationsLoadEmitter,
            this.onConfigurationsAddEmitter,
            this.onConfigurationChangeEmitter,
            this.onConfigurationRemoveEmitter
        ]);
    }

    protected get isReady(): Promise<void> {
        return this.ready.promise;
    }

    public async setConfiguration<V>(uuid: string, name: string, options: V): Promise<void> {
        const configuration = this.configurations.get(uuid);

        if (configuration) {
            configuration.name = name;
            configuration.options = options;

            this.fireConfigurationChange(configuration);
            return this.saveConfigurations();
        }
    }

    public async addConfiguration(configuration: Configuration): Promise<void> {
        await this.isReady;

        this.configurations.set(configuration.uuid, configuration);

        await this.saveConfigurations();

        this.fireConfigurationsAdd([configuration]);
    }

    public async addConfigurations(configurations: Configuration[]): Promise<void> {
        await this.isReady;

        configurations.forEach(configuration => this.configurations.set(configuration.uuid, configuration));

        await this.saveConfigurations();

        this.fireConfigurationsAdd(configurations);
    }

    public async removeConfiguration(uuid: string): Promise<void> {
        await this.isReady;

        this.configurations.delete(uuid);

        await this.saveConfigurations();

        this.fireConfigurationRemove(uuid);
    }

    public async getConfiguration(uuid: string): Promise<Configuration | undefined> {
        await this.isReady;
        return this.configurations.get(uuid);
    }

    public async getConfigurations(): Promise<Configuration[]> {
        await this.isReady;
        return [...this.configurations.values()];
    }

    protected async saveConfigurations(): Promise<void> {
        const uriConfigurations = await this.getConfigurationsFile();
        const configurations = await this.getConfigurations();
        const data = JSON.stringify(configurations);
        const stat = uriConfigurations && await this.fileSystem.getFileStat(uriConfigurations);

        if (stat) await this.fileSystem.setContent(stat, data);
        else if (uriConfigurations) await this.fileSystem.createFile(uriConfigurations, { content: data });
    }

    protected async loadConfigurations(): Promise<void> {
        this.ready = new Deferred();

        const uriConfigurations = await this.getConfigurationsFile();
        const uriExists = uriConfigurations && await this.fileSystem.exists(uriConfigurations);
        const contents = uriExists && uriConfigurations && await this.fileSystem.resolveContent(uriConfigurations);
        const configurations: Configuration[] = contents ? JSON.parse(contents.content) : [];

        this.configurations.clear();
        configurations.forEach(configuration => this.configurations.set(configuration.uuid, configuration));
        this.ready.resolve();
        this.fireConfigurationsLoad(configurations);
    }

    protected fireConfigurationsLoad(configurations: Configuration[]): void {
        return this.onConfigurationsLoadEmitter.fire(configurations);
    }

    protected fireConfigurationsAdd(configurations: Configuration[]): void {
        this.onConfigurationsAddEmitter.fire(configurations);
    }

    protected fireConfigurationChange(configuration: Configuration): void {
        this.onConfigurationChangeEmitter.fire(configuration);
    }

    protected fireConfigurationRemove(uuid: string): void {
        this.onConfigurationRemoveEmitter.fire(uuid);
    }

    protected async getConfigurationsFile(): Promise<string | undefined> {
        const workspace = await this.workspace.workspace;
        const uriWorkspace = workspace && new URI(workspace.uri);
        const uriConfigurationsFile = uriWorkspace && uriWorkspace.resolve(".theia/configurations.json");

        if (uriConfigurationsFile) return uriConfigurationsFile.toString();
    }

    public dispose(): void {
        this.toDispose.dispose();
    }

    public get onConfigurationsLoaded(): Event<Configuration[]> {
        return this.onConfigurationsLoadEmitter.event;
    }

    public get onConfigurationsAdded(): Event<Configuration[]> {
        return this.onConfigurationsAddEmitter.event;
    }

    public get onConfigurationChanged(): Event<Configuration> {
        return this.onConfigurationChangeEmitter.event;
    }

    public get onConfigurationRemoved(): Event<string> {
        return this.onConfigurationRemoveEmitter.event;
    }

    public async onStart(): Promise<void> {
        await this.loadConfigurations();
        this.toDispose.push(this.workspace.onWorkspaceChanged(() => this.loadConfigurations()));
    }
}
