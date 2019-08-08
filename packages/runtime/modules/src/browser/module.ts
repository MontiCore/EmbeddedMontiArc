/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
import { Emitter, Event } from "@theia/core/lib/common";
import URI from "@theia/core/lib/common/uri";
import { FileSystem } from "@theia/filesystem/lib/common";
import { ModulesService } from "./modules-service";

export type SourcesRootType = "sources" | "test-sources" | "generated-sources";

export type OutputRootType = "main" | "test";

export interface OutputRoot {
    uri: URI;
    type: OutputRootType;
}

export interface SourcesRoot {
    uri: URI;
    type: SourcesRootType;
}

export interface ModuleData {
    readonly name: string;
    readonly contentRoot: string;
    readonly sourcesRoots: RootData[];
    readonly outputRoots: RootData[];
    readonly dependencies: string[];
}

export interface RootData {
    readonly uri: string;
    readonly type: SourcesRootType | OutputRootType;
}

export interface Module {
    getModuleFile(): URI;
    setName(name: string): Promise<void>;
    getName(): string;
    setContentRoot(contentRoot: URI): Promise<void>;
    getContentRoot(): Promise<URI>;
    setSourcesRoots(sourcesRoots: SourcesRoot[]): Promise<void>;
    getSourcesRoots(type?: SourcesRootType): Promise<SourcesRoot[]>;
    setOutputRoots(outputRoots: OutputRoot[]): Promise<void>;
    getOutputRoots(type?: OutputRootType): Promise<OutputRoot[]>;
    getDependencies(): Promise<Module[]>;

    readonly onChanged: Event<ModuleChangeEvent>;
}

export const ModuleFactory = Symbol("ModuleFactory");
export interface ModuleFactory {
    (moduleFile: URI): Module;
}

export type ModuleAttribute = string | string[] | URI | SourcesRoot[] | OutputRoot[];

export interface ModuleChangeEvent {
    readonly attribute: string;
    readonly oldValue: ModuleAttribute;
    readonly newValue: ModuleAttribute;
}

export class ModuleImpl implements Module { // TODO: Optimize via caching.
    protected readonly fileSystem: FileSystem;
    protected readonly service: ModulesService;
    protected readonly onChangedEmitter: Emitter<ModuleChangeEvent>;

    protected moduleFile: URI;

    public constructor(moduleFile: URI, fileSystem: FileSystem, service: ModulesService) {
        this.moduleFile = moduleFile;
        this.fileSystem = fileSystem;

        this.onChangedEmitter = new Emitter<ModuleChangeEvent>();
    }

    public getModuleFile(): URI {
        return this.moduleFile;
    }

    public async setContentRoot(newContentRoot: URI): Promise<void> {
        const oldContentRoot = await this.getContentRoot();

        await this.setAttribute("contentRoot", newContentRoot);

        this.fireChangedEvent("contentRoot", oldContentRoot, newContentRoot);
    }

    public async getContentRoot(): Promise<URI> {
        const defaultValue = this.moduleFile.parent.toString();
        const uri = await this.getAttribute("contentRoot", defaultValue);

        return new URI(uri);
    }

    public async setName(newName: string): Promise<void> {
        const moduleFile = this.getModuleFile();
        const oldName = moduleFile.displayName;

        this.moduleFile = moduleFile.parent.resolve(`${newName}.json`);

        await this.fileSystem.move(moduleFile.toString(), this.moduleFile.toString());

        this.fireChangedEvent("name", oldName, newName);
    }

    public getName(): string {
        return this.moduleFile.displayName;
    }

    public async setOutputRoots(newOutputRoots: OutputRoot[]): Promise<void> {
        const oldOutputRoots = await this.getOutputRoots();
        const roots = newOutputRoots.map(root => this.serializeRoot(root));

        await this.setAttribute("outputRoots", roots);

        this.fireChangedEvent("outputRoots", oldOutputRoots, newOutputRoots);
    }

    public async getOutputRoots(type?: OutputRootType): Promise<OutputRoot[]> {
        const roots = await this.getAttribute<RootData[]>("outputRoots", []);
        const outputRoots = roots.map(data => this.deserializeRoot<OutputRoot>(data));

        return type ? outputRoots.filter(root => root.type === type) : outputRoots;
    }

    public async setSourcesRoots(newSourcesRoots: SourcesRoot[]): Promise<void> {
        const oldSourcesRoots = await this.getSourcesRoots();
        const roots = newSourcesRoots.map(root => this.serializeRoot(root));

        await this.setAttribute("sourcesRoots", roots);

        this.fireChangedEvent("sourcesRoots", oldSourcesRoots, newSourcesRoots);
    }

    public async getSourcesRoots(type?: SourcesRootType): Promise<SourcesRoot[]> {
        const roots = await this.getAttribute<RootData[]>("sourcesRoots", []);
        const sourcesRoots = roots.map(data => this.deserializeRoot<SourcesRoot>(data));

        return type ? sourcesRoots.filter(root => root.type === type) : sourcesRoots;
    }

    public async getDependencies(): Promise<Module[]> {
        const dependencies = await this.getAttribute<string[]>("dependencies", []);
        const promises = dependencies.map(name => this.service.getModule(name));
        const modules = await Promise.all(promises);

        return modules.filter(module => module !== undefined) as Module[];
    }

    public get onChanged(): Event<ModuleChangeEvent> {
        return this.onChangedEmitter.event;
    }

    protected async setAttribute<T>(attribute: string, value: T): Promise<void> {
        // tslint:disable-next-line:no-any
        const attributes = await this.getAttributes() as any;
        const fileStat = await this.fileSystem.getFileStat(this.moduleFile.toString());

        attributes[attribute] = value;

        await this.fileSystem.setContent(fileStat!, JSON.stringify(attributes));
    }

    protected async getAttribute<T>(attribute: string, defaultValue: T): Promise<T> {
        // tslint:disable-next-line:no-any
        const attributes = await this.getAttributes() as any;

        if (attributes.hasOwnProperty(attribute)) return attributes[attribute];
        else return defaultValue;
    }

    protected async getAttributes(): Promise<ModuleData> {
        let contents  = { content: "{}" };
        const uri = this.moduleFile.toString();

        if (await this.fileSystem.exists(uri)) contents = await this.fileSystem.resolveContent(uri);
        else await this.fileSystem.createFile(uri, contents);

        return JSON.parse(contents.content);
    }

    protected serializeRoot<T extends SourcesRoot | OutputRoot>(root: T): RootData {
        return {
            uri: root.uri.toString(),
            type: root.type
        };
    }

    protected deserializeRoot<T extends SourcesRoot | OutputRoot>(data: RootData): T {
        return {
            uri: new URI(data.uri),
            type: data.type
        } as T;
    }

    protected fireChangedEvent(attribute: string, oldValue: ModuleAttribute, newValue: ModuleAttribute): void {
        this.onChangedEmitter.fire({ attribute, oldValue, newValue });
    }
}

export namespace SourcesRoot {
    export function equals(root: SourcesRoot, peer: SourcesRoot): boolean {
        return root.uri === peer.uri && root.type === peer.type;
    }
}
