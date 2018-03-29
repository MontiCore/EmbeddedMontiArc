define(["require", "exports", "../se.rwth.common.module/module"], function (require, exports, module_1) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    class VFS extends module_1.Module {
        constructor(mountPoint) {
            super();
            this.mountPoint = mountPoint;
        }
        init(callback) {
            const system = {};
            const configuration = this.getConfiguration();
            const onConfigure = (error) => {
                this.fs = system.require("fs");
                callback(error);
            };
            BrowserFS.install(system);
            BrowserFS.configure(configuration, onConfigure);
        }
        getConfiguration() {
            return {
                fs: "IndexedDB",
                options: {
                    storeName: this.mountPoint
                }
            };
        }
        readFile(path, encoding = "utf8", callback = function () { }) {
            const normalizedPath = VFS.normalizeFile(path);
            this.fs.readFile(normalizedPath, encoding, callback);
        }
        readFileWithMetadata(path, options, callback = function () { }) {
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
        writeFile(path, data = '', encoding = "utf8", callback = function () { }, ensured = true) {
            const normalizedPath = VFS.normalizeFile(path);
            if (ensured)
                this.writeFileEnsured(normalizedPath, data, encoding, callback);
            else
                this.writeFileNonEnsured(normalizedPath, data, encoding, callback);
        }
        writeFileEnsured(path, data, encoding, callback) {
            const onMkdirP = (error) => {
                if (error)
                    callback(error);
                else
                    this.fs.writeFile(path, data, encoding, callback);
            };
            this.mkdirP(path, onMkdirP);
        }
        writeFileNonEnsured(path, data, encoding, callback) {
            this.fs.writeFile(path, data, encoding, callback);
        }
        appendFile(path, data = '', encoding = "utf8", callback = function () { }) {
            const normalizedPath = VFS.normalizeFile(path);
            this.fs.appendFile(normalizedPath, data, encoding, callback);
        }
        readdir(path, callback = function () { }) {
            const normalizedPath = VFS.normalizeDirectory(path);
            const onReaddir = (error, contents) => {
                if (error)
                    callback(error);
                else
                    this.readdirContents(normalizedPath, contents, [], callback);
            };
            this.fs.readdir(normalizedPath, onReaddir);
        }
        readdirContents(path, contents, data, callback) {
            const content = contents.shift();
            const onStat = (error, stat) => {
                data.push(stat);
                if (error)
                    callback(error);
                else
                    this.readdirContents(path, contents, data, callback);
            };
            if (content)
                this.stat(path + content, onStat);
            else
                callback(null, data);
        }
        exists(path, callback = function () { }) {
            const normalizedPath = VFS.normalizePath(path);
            this.fs.exists(normalizedPath, callback);
        }
        stat(path, callback = function () { }) {
            const normalizedPath = VFS.normalizePath(path);
            const onStat = (error, stat) => {
                if (error)
                    callback(error);
                else
                    VFS.handleStat(normalizedPath, stat, callback);
            };
            this.fs.stat(normalizedPath, onStat);
        }
        rename(from, to, options, callback = function () { }) {
            const normalizedFrom = VFS.normalizePath(from);
            const normalizedTo = VFS.normalizePath(to);
            this.fs.rename(normalizedFrom, normalizedTo, callback);
        }
        mkdirP(path, callback = function () { }) {
            const normalizedPath = VFS.normalizeDirectory(path);
            const chains = VFS.getSubPathsChains(normalizedPath);
            this.mkdirPChains(chains, callback);
        }
        mkdirPChains(chains, callback) {
            const chain = chains.shift();
            const onMkdir = (error) => {
                if (error && error !== "EEXIST")
                    callback(error);
                else
                    this.mkdirPChains(chains, callback);
            };
            if (chain)
                this.mkdir(chain, onMkdir);
            else
                callback(null);
        }
        mkdir(path, callback = function () { }) {
            const normalizedPath = VFS.normalizeDirectory(path);
            const extendedCallback = (error, result) => {
                callback(error, result);
                this.emit(normalizedPath, error, "directory", normalizedPath);
            };
            this.fs.mkdir(normalizedPath, extendedCallback);
        }
        unlink(path, callback = function () { }) {
            const normalizedPath = VFS.normalizePath(path);
            const extendedCallback = (error, result) => {
                callback(error, result);
                this.emit(normalizedPath, error, "change", normalizedPath);
            };
            this.fs.unlink(normalizedPath, extendedCallback);
        }
        rmfile(path, callback = function () { }) {
            const normalizedPath = VFS.normalizeFile(path);
            const extendedCallback = (error, result) => {
                callback(error, result);
                this.emit(normalizedPath, error, "change", normalizedPath);
            };
            this.fs.unlink(normalizedPath, extendedCallback);
        }
        rmdir(path, options, callback = function () { }) {
            const normalizedPath = VFS.normalizeDirectory(path);
            if (options.recursive)
                this.rmdirRecursive(normalizedPath, callback);
            else
                this.rmdirSingle(normalizedPath, callback);
        }
        rmdirSingle(path, callback = function () { }) {
            const extendedCallback = (error, result) => {
                callback(error, result);
                this.emit(path, error, "change", path);
            };
            this.fs.rmdir(path, extendedCallback);
        }
        rmdirRecursive(path, callback = function () { }) {
        }
        copy(from, to, options, callback = function () { }) {
            const onStat = (error, stat) => {
                if (error)
                    callback(error);
                else if (stat.mime === "folder")
                    this.copydir(from, to, options, callback);
                else
                    this.copyFile(from, to, options, callback);
            };
            this.stat(from, onStat);
        }
        copydir(from, to, options, callback) {
        }
        copyFile(from, to, options, callback) {
            if (options.overwrite)
                this.copyFileOverwrite(from, to, callback);
            else
                this.copyFileNoOverwrite(from, to, callback);
        }
        copyFileOverwrite(from, to, callback) {
            const onReadFile = (errorOuter, content) => {
                const onWriteFile = (errorInner) => {
                    callback(errorOuter || errorInner);
                };
                this.writeFile(to, content, "utf8", onWriteFile);
            };
            this.readFile(from, "utf8", onReadFile);
        }
        copyFileNoOverwrite(from, to, callback) {
        }
        chmod(path, mode, callback = function () { }) {
            const normalizedPath = VFS.normalizePath(path);
            this.fs.chmod(normalizedPath, mode, callback);
        }
        symlink(path, target, callback = function () { }) {
            const normalizedPath = VFS.normalizePath(path);
            const normalizedTarget = VFS.normalizePath(target);
            this.fs.symlink(normalizedPath, normalizedTarget, callback);
        }
        metadata(path, data, callback = function () { }) {
            const normalizedPath = VFS.normalizePath(path);
            const metadataPath = VFS.getMetadataPath(normalizedPath);
            this.writeFile(metadataPath, data, "utf8", callback);
        }
        watch(path, callback) {
            const normalizedPath = VFS.normalizePath(path);
            this.on(normalizedPath, callback);
            this.emit(normalizedPath, null, "init", normalizedPath);
        }
        unwatch(path, callback) {
            const normalizedPath = VFS.normalizePath(path);
            this.off(normalizedPath, callback);
        }
        static handleStat(path, stat, callback) {
            const data = this.normalizeStat(path, stat);
            callback(null, data);
        }
        static normalizeStat(path, stat) {
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
        static normalizeMTime(mtime) {
            return new Date(mtime).getTime();
        }
        static getMimeType(stat) {
            return stat.isDirectory() ? "folder" : "file";
        }
        static getBaseName(path) {
            const subPaths = path.split('/');
            const length = subPaths.length;
            return path.endsWith('/') ? subPaths[length - 2] : subPaths[length - 1];
        }
        static getSubPaths(path) {
            const length = path.length;
            const index = length - 1;
            const modifiedPath = path.substr(-1) === '/' ? path.substr(0, index) : path;
            return modifiedPath.split('/');
        }
        static getSubPathsChains(path) {
            const chains = [];
            const subPaths = VFS.getSubPaths(path);
            let chain = '';
            for (let subPath of subPaths) {
                chain += '/' + subPath;
                chains.push(chain);
            }
            return chains;
        }
        static getMetadataPath(path) {
            return "/.c9/metadata" + path;
        }
        static normalizePath(path) {
            const normalizedPath = path.charAt(0) === '/' ? path : '/' + path;
            return normalizedPath.replace('~', ".home");
        }
        static normalizeFile(path) {
            const normalizedPath = VFS.normalizePath(path);
            const length = normalizedPath.length;
            const index = length - 1;
            return normalizedPath.substr(-1) === '/' ? normalizedPath.substr(0, index) : normalizedPath;
        }
        static normalizeDirectory(path) {
            const normalizedPath = VFS.normalizePath(path);
            return normalizedPath.substr(-1) === '/' ? normalizedPath : normalizedPath + '/';
        }
    }
    exports.VFS = VFS;
});
//# sourceMappingURL=vfs.js.map