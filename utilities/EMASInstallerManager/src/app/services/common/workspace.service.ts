/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";

import * as path from "path";
import * as glob from "glob-promise";
import * as FileSystem from "fs-extra";

@Injectable({ providedIn: "root" })
export class WorkspaceService {
    public async getRoot(): Promise<string> {
        const rootPath = path.resolve(process.cwd(), "..");
        const rootPackage = path.join(rootPath, "package.json");
        const exists = await FileSystem.pathExists(rootPackage);
        const envVariable = process.env.EMASTUDIO_INSTALLER_ROOT;

        if (exists) return rootPath;
        else if (envVariable) return envVariable;
        else return '';
    }

    public async glob(relativePattern: string): Promise<string[]> {
        const rootPath = await this.getRoot();
        const absolutePattern = path.join(rootPath, relativePattern);

        return glob.promise(absolutePattern);
    }
}
