/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/// <reference types="@elysium/dashboard/src/typings/github-api" />

import { injectable, inject } from "inversify";
import * as GitHub from "github-api";
import * as JSZip from "jszip";
import { FileSystem } from "@theia/filesystem/lib/common";
import URI from "@theia/core/lib/common/uri";

export const DemosDownloader = Symbol("DemosDownloader");

export interface DemosDownloader {
    download(target: string, demo: string): Promise<void>;
}

@injectable()
export class DemosDownloaderImpl implements DemosDownloader {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;

    public async download(uri: string, demo: string): Promise<void> {
        const instance = new GitHub();
        const repository = instance.getRepo("EmbeddedMontiArc", "Demos");
        const root = await repository.getContents("master", `${demo}.zip`, false);
        const buffer = this.toBuffer(root.data.content);
        const contents = await JSZip.loadAsync(buffer);
        const files = contents.files;
        const paths = Object.getOwnPropertyNames(files);

        await Promise.all(
            paths.map(async path => {
                const file = files[path];
                const fileURI = new URI(uri).resolve(path).toString();

                if (file.dir) return this.fileSystem.createFolder(fileURI);

                const content = await file.async("text");

                return this.fileSystem.createFile(fileURI, { content });
            })
        );
    }

    protected toBuffer(string: string): ArrayBuffer {
        const binary = atob(string);
        const length = binary.length;
        const bytes = new Uint8Array(length);

        for (let i = 0; i < length; i++) {
            bytes[i] = binary.charCodeAt(i);
        }

        return bytes.buffer;
    }
}
