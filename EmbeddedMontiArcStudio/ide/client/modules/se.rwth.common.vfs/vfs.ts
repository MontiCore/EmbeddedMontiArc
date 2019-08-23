/* (c) https://github.com/MontiCore/monticore */
import {Callback, Module} from "../se.rwth.common.module/module";

declare var BrowserFS;

export class VFS extends Module {
    protected mountPoint: string;
    protected fs: any;


    public constructor(mountPoint: string) {
        super("common.vfs");
        this.mountPoint = mountPoint;
    }

    protected setDependencies(): void {}

    public init(callback: Callback): void {
        const system = <any>{};
        const configuration = this.getConfiguration();

        const onConfigure = (error) => {
            this.fs = system.require("fs");

            callback(error);
        };

        BrowserFS.install(system);
        BrowserFS.configure(configuration, onConfigure);
    }

    private getConfiguration(): any {
        return {
            fs: "IndexedDB",
            options: {
                storeName: this.mountPoint
            }
        };
    }


    public readFile(path: string, encoding: string = "utf8", callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizeFile(path);

        this.fs.readFile(normalizedPath, encoding, callback);
    }

    public readFileWithMetadata(path: string, options: any, callback: Callback = function() {}): void {
        const metadataPath = VFS.getMetadataPath(path);
        const encoding = options.encoding || "utf8";

        const onReadFile = (errorOuter, data) => {
            const onReadMetadataFile = (errorInner, metadata) => {
                callback(errorOuter, data, metadata);
            };

            this.readFile(metadataPath, encoding, onReadMetadataFile);
        };

        this.readFile(path, encoding, onReadFile);
    }

    public writeFile(path: string, data: any = '', encoding: string = "utf8",
                     callback: Callback = function() {}, ensured: boolean = true): void {
        const normalizedPath = VFS.normalizeFile(path);

        if(ensured) this.writeFileEnsured(normalizedPath, data, encoding, callback);
        else this.writeFileNonEnsured(normalizedPath, data, encoding, callback);
    }

    private writeFileEnsured(path: string, data: any, encoding: string, callback: Callback): void {
        const onMkdirP = (error) => {
            if(error) callback(error);
            else this.fs.writeFile(path, data, encoding, callback);
        };

        this.mkdirP(path, onMkdirP);
    }

    private writeFileNonEnsured(path: string, data: any, encoding: string, callback: Callback): void {
        this.fs.writeFile(path, data, encoding, callback);
    }

    public appendFile(path: string, data: any = '', encoding: string = "utf8",
                      callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizeFile(path);

        this.fs.appendFile(normalizedPath, data, encoding, callback);
    }

    public readdir(path: string, callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizeDirectory(path);

        const onReaddir = (error, contents) => {
            if(error) callback(error);
            else this.readdirContents(normalizedPath, contents, [], callback);
        };

        this.fs.readdir(normalizedPath, onReaddir);
    }

    private readdirContents(path: string, contents: string[], data: any[], callback: Callback): void {
        const content = contents.shift();

        const onStat = (error, stat) => {
            data.push(stat);

            if(error) callback(error);
            else this.readdirContents(path, contents, data, callback);
        };

        if(content) this.stat(path + content, onStat);
        else callback(null, data);
    }

    public exists(path: string, callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizePath(path);

        this.fs.exists(normalizedPath, callback);
    }

    public stat(path: string, callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizePath(path);

        const onStat = (error, stat) => {
            if(error) callback(error);
            else VFS.handleStat(normalizedPath, stat, callback);
        };

        this.fs.stat(normalizedPath, onStat);
    }

    public rename(from: string, to: string, options: any, callback: Callback = function() {}): void {
        const normalizedFrom = VFS.normalizePath(from);
        const normalizedTo = VFS.normalizePath(to);

        this.fs.rename(normalizedFrom, normalizedTo, callback);
    }

    public mkdirP(path: string, callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizeDirectory(path);
        const chains = VFS.getSubPathsChains(normalizedPath);

        this.mkdirPChains(chains, callback);
    }

    private mkdirPChains(chains: string[], callback: Callback): void {
        const chain = chains.shift();

        const onMkdir = (error) => {
            if(error && error !== "EEXIST") callback(error);
            else this.mkdirPChains(chains, callback);
        };

        if(chain) this.mkdir(chain, onMkdir);
        else callback(null);
    }

    public mkdir(path: string, callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizeDirectory(path);

        const extendedCallback = (error, result) => {
            callback(error, result);
            this.emit(normalizedPath, error, "directory", normalizedPath);
        };

        this.fs.mkdir(normalizedPath, extendedCallback);
    }

    public unlink(path: string, callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizePath(path);

        const extendedCallback = (error, result) => {
            callback(error, result);
            this.emit(normalizedPath, error, "change", normalizedPath);
        };

        this.fs.unlink(normalizedPath, extendedCallback);
    }

    public rmfile(path: string, callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizeFile(path);

        const extendedCallback = (error, result) => {
            callback(error, result);
            this.emit(normalizedPath, error, "change", normalizedPath);
        };

        this.fs.unlink(normalizedPath, extendedCallback);
    }

    public rmdir(path: string, options: any, callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizeDirectory(path);

        if(options.recursive) this.rmdirRecursive(normalizedPath, callback);
        else this.rmdirSingle(normalizedPath, callback);
    }

    public rmdirSingle(path: string, callback: Callback = function() {}): void {
        const extendedCallback = (error, result) => {
            callback(error, result);
            this.emit(path, error, "change", path);
        };

        this.fs.rmdir(path, extendedCallback);
    }

    public rmdirRecursive(path: string, callback: Callback = function() {}): void {

    }

    public copy(from: string, to: string, options: any, callback: Callback = function() {}): void {
        const onStat = (error, stat) => {
            if(error) callback(error);
            else if(stat.mime === "folder") this.copydir(from, to, options, callback);
            else this.copyFile(from, to, options, callback);
        };

        this.stat(from, onStat);
    }

    private copydir(from: string, to: string, options: any, callback: Callback): void {

    }

    private copyFile(from: string, to: string, options: any, callback: Callback): void {
        if(options.overwrite) this.copyFileOverwrite(from, to, callback);
        else this.copyFileNoOverwrite(from, to, callback);
    }

    private copyFileOverwrite(from: string, to: string, callback: Callback): void {
        const onReadFile = (errorOuter, content) => {
            const onWriteFile = (errorInner) => {
                callback(errorOuter || errorInner);
            };

            this.writeFile(to, content, "utf8", onWriteFile);
        };

        this.readFile(from, "utf8", onReadFile);
    }

    private copyFileNoOverwrite(from: string, to: string, callback: Callback): void {

    }

    public chmod(path: string, mode: number, callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizePath(path);

        this.fs.chmod(normalizedPath, mode, callback);
    }

    public symlink(path: string, target: string, callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizePath(path);
        const normalizedTarget = VFS.normalizePath(target);

        this.fs.symlink(normalizedPath, normalizedTarget, callback);
    }

    public metadata(path: string, data: any, callback: Callback = function() {}): void {
        const normalizedPath = VFS.normalizePath(path);
        const metadataPath = VFS.getMetadataPath(normalizedPath);

        this.writeFile(metadataPath, data, "utf8", callback);
    }

    public watch(path: string, callback: Callback): void {
        const normalizedPath = VFS.normalizePath(path);

        this.on(normalizedPath, callback);
        this.emit(normalizedPath, null, "init", normalizedPath);
    }

    public unwatch(path: string, callback: Callback): void {
        const normalizedPath = VFS.normalizePath(path);

        this.off(normalizedPath, callback);
    }


    private static handleStat(path: string, stat: any, callback: Callback): void {
        const data = this.normalizeStat(path, stat);

        callback(null, data);
    }

    private static normalizeStat(path: string, stat: any): any {
        return {
            name: VFS.getBaseName(path),
            fullPath: path,
            size: stat.size,
            mtime: VFS.normalizeMTime(stat.mtime),
            mime: VFS.getMimeType(stat),
            link: false,
            linkStat: {}
        };
    }

    private static normalizeMTime(mtime: number): number {
        return new Date(mtime).getTime();
    }

    private static getMimeType(stat: any): string {
        return stat.isDirectory() ? "folder" : "file";
    }

    private static getBaseName(path: string): string {
        const subPaths = path.split('/');
        const length = subPaths.length;

        return path.endsWith('/') ? subPaths[length - 2] : subPaths[length - 1];
    }

    private static getSubPaths(path: string): string[] {
        const length = path.length;
        const index = length - 1;
        const modifiedPath = path.substr(-1) === '/' ? path.substr(0, index) : path;

        return modifiedPath.split('/');
    }

    private static getSubPathsChains(path: string): string[] {
        const chains = [];
        const subPaths = VFS.getSubPaths(path);

        let chain = '';

        for(let subPath of subPaths) {
            chain += '/' + subPath;

            chains.push(chain);
        }

        return chains;
    }

    private static getMetadataPath(path: string): string {
        return "/.c9/metadata" + path;
    }

    private static normalizePath(path: string): string {
        const normalizedPath = path.charAt(0) === '/' ? path : '/' + path;

        return normalizedPath.replace('~', ".home");
    }

    private static normalizeFile(path: string): string {
        const normalizedPath = VFS.normalizePath(path);
        const length = normalizedPath.length;
        const index = length - 1;

        return normalizedPath.substr(-1) === '/' ? normalizedPath.substr(0, index) : normalizedPath;
    }

    private static normalizeDirectory(path: string): string {
        const normalizedPath = VFS.normalizePath(path);

        return normalizedPath.substr(-1) === '/' ? normalizedPath : normalizedPath + '/';
    }
}
