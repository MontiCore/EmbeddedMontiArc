/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { BackendApplicationContribution } from "@theia/core/lib/node/backend-application";
import { CLUSTERING_EXEC_PATH_ID, CLUSTERING_PATH } from "../common";
import { Application, Request, Response } from "express";
import { FileSystem } from "@theia/filesystem/lib/common";
import { FileUri } from "@theia/core/lib/node/file-uri";
import { PathsRegistry } from "@emastudio/paths/lib/node";
import { FileArray, UploadedFile } from "express-fileupload";

import * as FileUpload from "express-fileupload";

import Jimp = require("jimp");

@injectable()
export class ClusteringBackendContribution implements BackendApplicationContribution {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;
    @inject(PathsRegistry) protected readonly pathsRegistry: PathsRegistry;

    public configure(application: Application): void {
        application.use(CLUSTERING_PATH, FileUpload());
        application.post(CLUSTERING_PATH, this.onPostRequest.bind(this));
    }

    protected async onPostRequest(request: Request, response: Response): Promise<void> {
        const files = request.files;

        if (files) return this.handlePostRequest(files, response);
        else response.end();
    }

    protected async handlePostRequest(files: FileArray, response: Response): Promise<void> {
        const uri = await this.pathsRegistry.getPath(CLUSTERING_EXEC_PATH_ID);
        const destination = FileUri.fsPath(uri.resolve("img.bmp"));
        const file = files["file"] as UploadedFile;

        await file.mv(destination);

        const image = await Jimp.read(destination);

        await image.resize(50, 50).write(destination);
        response.end();
    }
}
