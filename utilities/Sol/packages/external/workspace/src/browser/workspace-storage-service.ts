/*
 * (c) https://github.com/MontiCore/monticore
 */
import { StorageService } from "@theia/core/lib/browser";
import { FileSystem } from "@theia/filesystem/lib/common/filesystem";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { inject, injectable, postConstruct } from "inversify";

import URI from "@theia/core/lib/common/uri";

@injectable()
export class WorkspaceStorageService implements StorageService {
    @inject(WorkspaceService) protected readonly workspace: WorkspaceService;
    @inject(FileSystem) protected readonly fs: FileSystem;

    protected data: any; // tslint:disable-line:no-any
    protected initialized: Promise<void>;
    protected storageFile: URI | undefined;

    @postConstruct()
    protected init(): void {
        this.initialized = this.workspace.roots.then(() => {
            this.updateStorageFile();
            this.workspace.onWorkspaceChanged(() => this.updateStorageFile());
            return this.load();
        });
    }

    public async getData<T>(key: string, defaultValue?: T): Promise<T | undefined> {
        await this.initialized;

        if (this.data.hasOwnProperty(key)) return this.data[key];
        else return defaultValue;
    }

    public async setData<T>(key: string, data: T): Promise<void> {
        await this.initialized;

        this.data[key] = data;
        return this.save();
    }

    protected async save(): Promise<void> {
        if (this.storageFile) return this.doSave(this.storageFile.toString());
        else throw new Error(`Could not locate workspace storage file.`);
    }

    protected async doSave(uri: string): Promise<void> {
        const data = JSON.stringify(this.data);
        const stat = await this.fs.getFileStat(uri);

        if (stat) await this.fs.setContent(stat, data);
        else await this.fs.createFile(uri, { content: data });
    }

    protected async load(): Promise<void> {
        if (this.storageFile) return this.doLoad(this.storageFile.toString());
        else throw new Error(`Could not locate workspace storage file.`);
    }

    protected async doLoad(uri: string): Promise<void> {
        let contents = { content: "{}" };
        const stat = await this.fs.getFileStat(uri);

        if (stat) contents = await this.fs.resolveContent(uri);

        this.data = JSON.parse(contents.content);
    }

    protected updateStorageFile(): void {
        const workspace = this.workspace.workspace;

        if (workspace) this.storageFile = new URI(workspace.uri).resolve(".theia/storage.json");
    }
}
