/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/* tslint:disable:no-null-keyword */

import IndexedDBFileSystem from "browserfs/dist/node/backend/IndexedDB";
import { File } from "browserfs/dist/node/core/file";
import { FileFlag } from "browserfs/dist/node/core/file_flag";
import Stats from "browserfs/dist/node/core/node_fs_stats";
import { configure } from "./browserfs-extra";
import URI from "@elysium/core/lib/common/uri";

let fs: IndexedDBFileSystem;

async function createFileSystem(): Promise<void> {
    if (!fs) return doCreateFileSystem();
}

async function doCreateFileSystem(): Promise<void> {
    const module = await configure({ fs: "IndexedDB", options: { storeName: "Elysium" } });

    if (module) fs = module.getRootFS() as IndexedDBFileSystem;
    else throw new Error("Virtual File System could not be created!");
}

export async function createFile(p: string, flag: FileFlag, mode: number): Promise<File> {
    await createFileSystem();

    return new Promise<File>((resolve, reject) => {
        fs.createFile(p, flag, mode, (error, file) => {
            if (error) reject(error);
            else resolve(file);
        });
    });
}

export async function exists(p: string): Promise<boolean> {
    await createFileSystem();

    return new Promise<boolean>(resolve => {
        fs.exists(p, existence => {
            resolve(existence);
        });
    });
}

export async function mkdir(p: string, mode: number): Promise<void> {
    await createFileSystem();

    return new Promise<void>((resolve, reject) => {
        fs.mkdir(p, mode, error => {
            if (error) reject(error);
            else resolve();
        });
    });
}

export async function readFile(fname: string, encoding: string | null, flag: FileFlag): Promise<string | Buffer> {
    await createFileSystem();

    return new Promise<string | Buffer>((resolve, reject) => {
        fs.readFile(fname, encoding, flag, (error, content) => {
            if (error) reject(error);
            else resolve(content);
        });
    });
}

export async function readdir(p: string): Promise<string[]> {
    await createFileSystem();

    return new Promise<string[]>((resolve, reject) => {
        fs.readdir(p, (error, contents) => {
            if (error) reject(error);
            else resolve(contents);
        });
    });
}

export async function rename(oldPath: string, newPath: string): Promise<void> {
    await createFileSystem();

    return new Promise<void>((resolve, reject) => {
        fs.rename(oldPath, newPath, error => {
            if (error) reject(error);
            else resolve();
        });
    });
}

export async function rmdir(p: string): Promise<void> {
    await createFileSystem();

    return new Promise<void>((resolve, reject) => {
        fs.rmdir(p, error => {
            if (error) reject(error);
            else resolve();
        });
    });
}

export async function stat(p: string, isLstat: boolean): Promise<Stats> {
    await createFileSystem();

    return new Promise<Stats>((resolve, reject) => {
        fs.stat(p, isLstat, (error, stats) => {
            if (error) reject(error);
            else resolve(stats);
        });
    });
}

export async function unlink(p: string): Promise<void> {
    await createFileSystem();

    return new Promise<void>((resolve, reject) => {
        fs.unlink(p, error => {
            if (error) reject(error);
            else resolve();
        });
    });
}

// tslint:disable-next-line:no-any
export async function writeFile(fname: string, data: any, encoding: string | null, flag: FileFlag, mode: number): Promise<void> {
    await createFileSystem();

    return new Promise<void>((resolve, reject) => {
        fs.writeFile(fname, data, encoding, flag, mode, error => {
            if (error) reject(error);
            else resolve();
        });
    });
}

export async function mkdirp(p: string): Promise<void> {
    const uri = new URI(p);
    const uris = uri.getSubURIs();

    for (const u of uris) {
        try {
            await mkdir(u.toString(), 0x1a4);
        } catch (error) {
            if (error.code !== "EEXIST") throw error;
        }
    }
}

export async function touch(p: string): Promise<void> {
    const flag = FileFlag.getFileFlag("w+");
    const content = await readFile(p, null, flag);

    return writeFile(p, content, null, flag, 0x1a4);
}

export async function remove(p: string): Promise<void> {
    const vstat = await stat(p, false);

    if (vstat.isDirectory()) return rmdirr(p);
    else return unlink(p);
}

async function rmdirr(p: string): Promise<void> {
    const contents = await readdir(p);
    const uri = new URI(p);

    for (const content of contents) {
        const contentURI = uri.resolve(content).toString();
        const vstat = await stat(contentURI, false);

        if (vstat.isDirectory()) await rmdirr(contentURI);
        else await unlink(contentURI);
    }

    return rmdir(p);
}

export async function copy(source: string, target: string, options?: { overwrite?: boolean, recursive?: boolean }): Promise<void> {
    const targetExists = await exists(target);
    const condition = (targetExists && options && options.overwrite) || !targetExists;

    if (condition) return doCopy(source, target, options);
}

async function doCopy(source: string, target: string, options?: { overwrite?: boolean, recursive?: boolean }): Promise<void> {
    const vstat = await stat(source, false);
    const parent = new URI(target).parent.toString();

    await mkdirp(parent);

    if (vstat.isFile()) return doCopyFile(source, target);
    else return doCopyDirectory(source, target, options);
}

async function doCopyFile(source: string, target: string): Promise<void> {
    const readFlag = FileFlag.getFileFlag("r+");
    const writeFlag = FileFlag.getFileFlag("w+");
    const content = await readFile(source, "utf-8", readFlag);

    return writeFile(target, content, "utf-8", writeFlag, 0x1a4);
}

async function doCopyDirectory(source: string, target: string, options?: { overwrite?: boolean, recursive?: boolean }): Promise<void> {
    const contents = await readdir(source);
    const sourceURI = new URI(source);
    const targetURI = new URI(target);

    await mkdir(target, 0x1a4);

    for (const content of contents) {
        const contentSource = sourceURI.resolve(content).toString();
        const contentTarget = targetURI.resolve(content).toString();
        const vstat = await stat(contentSource, false);

        if (vstat.isFile()) await doCopyFile(contentSource, contentTarget);
        else if (options && options.recursive) await copy(contentSource, contentTarget, options);
        else await mkdir(contentTarget, 0x1a4);
    }
}

export async function move(source: string, target: string, options?: { overwrite?: boolean }): Promise<void> {
    const targetExists = await exists(target);
    const condition = (targetExists && options && options.overwrite) || !targetExists;

    if (condition) return doMove(source, target);
    else throw new Error(`Error occurred while moving the file. The file does already exist at ${target}.`);
}

async function doMove(source: string, target: string, options?: { overwrite?: boolean }): Promise<void> {
    const vstat = await stat(source, false);

    if (vstat.isFile()) return doMoveFile(source, target);
    else return doMoveDirectory(source, target, options);
}

async function doMoveFile(source: string, target: string): Promise<void> {
    const readFlag = FileFlag.getFileFlag("r+");
    const writeFlag = FileFlag.getFileFlag("w+");
    const content = await readFile(source, "utf-8", readFlag);

    await writeFile(target, content, "utf-8", writeFlag, 0x1a4);
    return remove(source);
}

async function doMoveDirectory(source: string, target: string, options?: { overwrite?: boolean }): Promise<void> {
    const sourceURI = new URI(source);
    const targetURI = new URI(target);
    const contents = await readdir(source);

    await mkdir(target, 0x1a4);

    for (const content of contents) {
        const contentSource = sourceURI.resolve(content).toString();
        const contentTarget = targetURI.resolve(content).toString();

        await move(contentSource, contentTarget, options);
    }

    return rmdir(source);
}
