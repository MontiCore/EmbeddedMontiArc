/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import {inject, injectable} from "inversify";
import { DownloadConfig, Downloader } from "./downloader";
import { JSZipObject } from "jszip";
import * as JSZip from "jszip";
import URI from "@elysium/core/lib/common/uri";
import { FileSystem, FileStat } from "@theia/filesystem/lib/common";

@injectable()
export class ZIPDownloader implements Downloader {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;

    public async download(config: DownloadConfig): Promise<void> {
        if (config.remoteURI) {
            const localURI = new URI(config.localURI);
            const response = await fetch(config.remoteURI);
            const zipFile = await response.arrayBuffer();
            const contents = await JSZip.loadAsync(zipFile);

            return this.handleFiles(localURI, contents.files);
        } else {
            console.warn("[ZIPDownloader]: The specification of a remote uri is mandatory.");
        }
    }

    protected async handleFiles(localURI: URI, files: { [p: string]: JSZipObject }): Promise<void> {
        const paths = Object.getOwnPropertyNames(files);
        const promises = paths.map(path => this.handleFile(localURI, files[path]));

        await Promise.all(promises);
    }

    protected async handleFile(localURI: URI, file: JSZipObject): Promise<FileStat> {
        const fileURI = localURI.resolve(file.name).toString();

        if (file.dir) return this.fileSystem.createFolder(fileURI);

        const content = await file.async("text");

        return this.fileSystem.createFile(fileURI, { content });
    }
}
