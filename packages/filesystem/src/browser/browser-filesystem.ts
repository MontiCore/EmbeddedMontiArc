/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import URI from '@theia/core/lib/common/uri';
import { FileStat, FileSystem, FileSystemClient } from "@theia/filesystem/lib/common/filesystem";
import { FileSystemWatcherServer } from "@theia/filesystem/lib/common/filesystem-watcher-protocol";
import { ApiError as APIError } from "browserfs/dist/node/core/api_error";
import { FileFlag } from "browserfs/dist/node/core/file_flag";
import VFileStat from "browserfs/dist/node/core/node_fs_stats";
import { inject, injectable } from "inversify";
import { BrowserFileSystemWatcher } from "./browser-filesystem-watcher";
import * as fs from "./fs-extra";
import { TextDocumentContentChangeEvent, TextDocument } from "vscode-languageserver-types";
import { FileUri } from "@theia/core/lib/node/file-uri";

@injectable()
export class FileSystemBrowser implements FileSystem {
    protected client: FileSystemClient | undefined;
    protected watcher: BrowserFileSystemWatcher;

    public constructor(
        @inject(FileSystemWatcherServer) watcher: FileSystemWatcherServer
    ) {
        this.watcher = watcher as BrowserFileSystemWatcher;
    }

    public setClient(client: FileSystemClient | undefined): void {
        this.client = client;
    }

    public async getFileStat(uri: string): Promise<FileStat> {
        const stat = await this.doGetStat(uri, 1);

        if (stat) return stat;
        else throw new Error(`Cannot find file under the given URI. URI: ${uri}.`);
    }

    protected async doGetStat(uri: string, depth: number): Promise<FileStat | undefined> {
        try {
            return await this.handleGetStat(uri, depth);
        } catch (error) {
            return await this.handleGetStatError(error);
        }
    }

    protected async handleGetStat(uri: string, depth: number): Promise<FileStat> {
        const vstat = await fs.stat(FileUri.fsPath(uri), false);

        return vstat.isDirectory() ?
            this.doCreateDirectoryStat(uri, vstat, depth) :
            this.doCreateFileStat(uri, vstat);
    }

    protected async handleGetStatError(error: APIError): Promise<FileStat | undefined> {
        if (error.code === "ENOENT" || error.code === "EACCES" || error.code === "EBUSY" || error.code === "EPERM") return undefined;
        else throw error;
    }

    protected doCreateFileStat(uri: string, vstat: VFileStat): FileStat {
        return {
            isDirectory: false,
            lastModification: vstat.mtime.getTime(),
            size: vstat.size,
            uri
        };
    }

    protected async doCreateDirectoryStat(uri: string, vstat: VFileStat, depth: number): Promise<FileStat> {
        const children = depth > 0 ? await this.doGetChildren(uri, depth) : [];

        return {
            children,
            isDirectory: true,
            lastModification: vstat.mtime.getTime(),
            uri
        };
    }

    protected async doGetChildren(uri: string, depth: number): Promise<FileStat[]> {
        const u = new URI(uri);
        const contents = await fs.readdir(FileUri.fsPath(uri));
        const children = await Promise.all(
            contents
                .map(content => u.resolve(content).toString())
                .map(path => this.doGetStat(path, depth - 1))
        );

        function notEmpty<T>(value: T | undefined): value is T {
            return value !== undefined;
        }

        return children.filter(notEmpty);
    }

    public async exists(uri: string): Promise<boolean> {
        return fs.exists(FileUri.fsPath(uri));
    }

    public async resolveContent(uri: string, options?: { encoding?: string }): Promise<{ stat: FileStat, content: string }> {
        const stat = await this.doGetStat(uri, 0);

        if (!stat) throw new Error(`Cannot find file under the given URI. URI: ${uri}.`);
        if (stat.isDirectory) throw new Error(`Cannot resolve the content of a directory. URI: ${uri}.`);

        const encoding = this.doGetEncoding(options);
        const flag = FileFlag.getFileFlag("r+");
        const content = await fs.readFile(FileUri.fsPath(uri), encoding, flag) as string;

        return { stat, content };
    }

    protected doGetEncoding(options?: { encoding?: string }): string {
        return options && typeof (options.encoding) !== "undefined" ? options.encoding : "utf-8";
    }

    public async setContent(file: FileStat, content: string, options?: { encoding?: string }): Promise<FileStat> {
        const uri = file.uri;
        const stat = await this.doGetStat(uri, 0);

        if (!stat) throw new Error(`Cannot find file under the given URI. URI: ${uri}.`);
        if (stat.isDirectory) throw new Error(`Cannot set the content of a directory. URI: ${uri}.`);

        const encoding = this.doGetEncoding(options);
        const flag = FileFlag.getFileFlag("w+");

        await fs.writeFile(FileUri.fsPath(uri), content, encoding, flag, 0x1a4);

        const newStat = await this.doGetStat(uri, 1);

        if (newStat) {
            this.watcher.addModified(uri);

            return newStat;
        } else {
            throw new Error(`Error occurred while writing file content. The file does not exist under ${uri}.`);
        }
    }

    public async updateContent(file: FileStat, contentChanges: TextDocumentContentChangeEvent[], options?: { encoding?: string }): Promise<FileStat> {
        const uri = file.uri;
        const stat = await this.doGetStat(uri, 0);

        if (!stat) throw new Error(`Cannot find file under the given URI. URI: ${file.uri}.`);
        if (stat.isDirectory) throw new Error(`Cannot set the content of a directory. URI: ${file.uri}.`);
        if (contentChanges.length === 0) return stat;

        const encoding = await this.doGetEncoding(options);
        const readFlag = FileFlag.getFileFlag("r+");
        const content = await fs.readFile(FileUri.fsPath(uri), encoding, readFlag);

        const writeFlag = FileFlag.getFileFlag("w+");
        const newContent = this.applyContentChanges(content.toString(), contentChanges);

        await fs.writeFile(FileUri.fsPath(uri), newContent, encoding, writeFlag, 0x1a4);

        const newStat = await this.doGetStat(uri, 0);

        if (newStat) {
            this.watcher.addModified(uri);

            return newStat;
        } else {
            throw new Error(`Error occurred while updating file content. The file does not exist under ${uri}.`);
        }
    }

    protected applyContentChanges(content: string, contentChanges: TextDocumentContentChangeEvent[]): string {
        let document = TextDocument.create('', '', 1, content);

        for (const contentChange of contentChanges) {
            let newContent = contentChange.text;
            const range = contentChange.range;

            if (range) {
                const start = document.offsetAt(range.start);
                const end = document.offsetAt(range.end);

                newContent = document.getText().substr(0, start) + newContent + document.getText().substr(end);
            }

            document = TextDocument.create(document.uri, document.languageId, document.version, newContent);
        }

        return document.getText();
    }

    public async move(sourceUri: string, targetUri: string, options?: { overwrite?: boolean }): Promise<FileStat> {
        await fs.move(FileUri.fsPath(sourceUri), FileUri.fsPath(targetUri), options);

        const stat = await this.doGetStat(targetUri, 1);

        if (stat) {
            this.watcher.addDeleted(sourceUri);
            this.watcher.addCreated(targetUri);

            return stat;
        } else {
            throw new Error(`Error occurred while moving the file. The file does not exist at ${targetUri}.`);
        }
    }

    public async copy(sourceUri: string, targetUri: string, options?: { overwrite?: boolean, recursive?: boolean }): Promise<FileStat> {
        await fs.copy(FileUri.fsPath(sourceUri), FileUri.fsPath(targetUri), options);

        const stat = await this.doGetStat(targetUri, 1);

        if (stat) {
            this.watcher.addCreated(targetUri);

            return stat;
        } else {
            throw new Error(`Error occurred while copying the file. The file does not exist at ${targetUri}.`);
        }
    }

    public async createFile(uri: string, options?: { content?: string, encoding?: string }): Promise<FileStat> {
        const urio = new URI(uri);
        const parentURI = urio.parent.toString();
        const [stat, parentStat] = await Promise.all([
            this.doGetStat(uri, 0),
            this.doGetStat(parentURI, 0)
        ]);

        if (stat) throw new Error(`Error occurred while creating the file. File already exists at ${uri}.`);
        if (!parentStat) await fs.mkdirp(FileUri.fsPath(parentURI));

        const content = await this.doGetContent(options);
        const encoding = this.doGetEncoding(options);
        const flag = FileFlag.getFileFlag("w+");

        await fs.writeFile(FileUri.fsPath(uri), content, encoding, flag, 0x1a4);

        const newStat = await this.doGetStat(uri, 1);

        if (newStat) {
            this.watcher.addCreated(uri);

            return newStat;
        } else {
            throw new Error(`Error occurred while creating new file. The file does not exist at ${uri}.`);
        }
    }

    protected doGetContent(options?: { content?: string }): string {
        return (options && options.content) || '';
    }

    public async createFolder(uri: string): Promise<FileStat> {
        const stat = await this.doGetStat(uri, 0);

        if (stat) throw new Error(`Error occurred while creating the directory. File already exists at ${uri}.`);

        await fs.mkdirp(FileUri.fsPath(uri));

        const newStat = await this.doGetStat(uri, 1);

        if (newStat) {
            this.watcher.addCreated(uri);

            return newStat;
        } else {
            throw new Error(`Error occurred while creating the directory. The directory does not exist at ${uri}.`);
        }
    }

    public async touchFile(uri: string): Promise<FileStat> {
        const stat = await this.doGetStat(uri, 0);

        if (stat) await fs.touch(FileUri.fsPath(uri));
        else return await this.createFile(uri);

        const newStat = await this.doGetStat(uri, 1);

        if (newStat) return newStat;
        else throw new Error(`Error occurred while touching the file. The file could not be touched at ${uri}.`);
    }

    public async delete(uri: string, options?: { moveToTrash?: boolean }): Promise<void> {
        // TODO: Implement trash bin.
        const stat = await this.doGetStat(uri, 0);

        if (!stat) throw new Error(`File does not exist under ${uri}.`);

        const moveToTrash = this.doGetMoveToTrash(options);

        if (moveToTrash) {
            /* NOOP */
            console.log("Would be moved to the trash bin.");
        } else {
            await fs.remove(FileUri.fsPath(uri));
            this.watcher.addDeleted(uri);
        }
    }

    protected doGetMoveToTrash(options?: { moveToTrash?: boolean }): boolean {
        return options && typeof (options.moveToTrash) !== 'undefined' ? options.moveToTrash : false;
    }

    public async getEncoding(uri: string): Promise<string> {
        const stat = await this.doGetStat(uri, 0);

        if (!stat) throw new Error(`File does not exist under ${uri}.`);
        if (stat.isDirectory) throw new Error(`Cannot get the encoding of a directory. URI: ${uri}.`);

        return "utf-8";
    }

    protected static readonly ROOT_DIRECTORY: string = '/';

    public async getRoots(): Promise<FileStat[]> {
        const rootUri = FileUri.create(FileSystemBrowser.ROOT_DIRECTORY).toString();
        const root = await this.doGetStat(rootUri, 1);

        if (root) return [root];
        else return [];
    }

    protected static readonly USER_HOME: string = '/';

    public async getCurrentUserHome(): Promise<FileStat | undefined> {
        const uri = FileUri.create(FileSystemBrowser.USER_HOME).toString();
        /*const exist = await this.exists(uri);

        if (!exist) await fs.mkdirp(uri);*/

        return this.getFileStat(uri);
    }

    public dispose(): void {
        // NOOP
    }
}
