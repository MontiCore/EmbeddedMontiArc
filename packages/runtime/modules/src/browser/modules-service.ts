/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
import URI from "@theia/core/lib/common/uri";
import { FileStat, FileSystem } from "@theia/filesystem/lib/common";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { inject, injectable, postConstruct } from "inversify";
import { Module, ModuleFactory } from "./module";

export const ModulesService = Symbol("ModulesService");
export interface ModulesService {
    addModule(module: Module): Promise<void>;
    removeModule(name: string): Promise<void>;
    getModule(name: string): Promise<Module | undefined>;
    getModules(): Promise<Module[]>;
}

@injectable()
export class ModulesServiceImpl implements ModulesService {
    @inject(WorkspaceService) protected readonly workspace: WorkspaceService;
    @inject(FileSystem) protected readonly fileSystem: FileSystem;
    @inject(ModuleFactory) protected readonly factory: ModuleFactory;

    protected isReady: Promise<Module[]>;
    protected modules: Module[];

    @postConstruct()
    protected async init(): Promise<void> {
        const workspace = await this.workspace.workspace;

        if (workspace) this.isReady = this.fetchModules(workspace);
        else this.isReady = Promise.resolve([]);

        this.modules = await this.isReady;
    }

    public async addModule(module: Module): Promise<void> {
        await this.isReady;

        this.modules.push(module);

        return this.storeModules();
    }

    public async removeModule(name: string): Promise<void> {
        await this.isReady;

        const module = await this.getModule(name);

        if (module) {
            this.modules.splice(this.modules.indexOf(module), 1);
            return this.storeModules();
        }
    }

    public async getModule(name: string): Promise<Module | undefined> {
        await this.isReady;
        return this.modules.find(module => module.getName() === name);
    }

    public async getModules(): Promise<Module[]> {
        await this.isReady;
        return this.modules;
    }

    protected async storeModules(): Promise<void> {
        const workspace = await this.workspace.workspace;

        if (workspace) {
            const workspaceURI = new URI(workspace.uri);
            const fileStat = await this.getModulesFileStat(workspace);
            const toRelativeURI = (module: Module) => module.getModuleFile().relative(workspaceURI);
            const paths = this.modules.map(toRelativeURI).map(uri => uri!.toString());

            await this.fileSystem.setContent(fileStat, JSON.stringify(paths));
        }
    }

    protected async fetchModules(workspace: FileStat): Promise<Module[]> {
        const workspaceURI = new URI(workspace.uri);
        const fileStat = await this.getModulesFileStat(workspace);
        const contents = await this.fileSystem.resolveContent(fileStat.uri);
        const paths = JSON.parse(contents.content) as string[];
        const toAbsolutePath = (path: string) => workspaceURI.resolve(path);
        const toModule = (moduleFile: URI) => this.factory(moduleFile);

        return paths.map(toAbsolutePath).map(toModule);
    }

    protected async getModulesFileStat(workspace: FileStat): Promise<FileStat> {
        const workspaceURI = new URI(workspace.uri);
        const modulesURI = workspaceURI.resolve(".theia").resolve("modules.json").toString();
        const fileStat = await this.fileSystem.getFileStat(modulesURI);

        if (fileStat) return fileStat;
        else return this.fileSystem.createFile(modulesURI, { content: "[]" });
    }
}
