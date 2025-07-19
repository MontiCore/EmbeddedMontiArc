/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";
import { WorkspaceService } from "@services/common/workspace.service";
import { LoadableService } from "@services/common/load.service";

import * as path from "path";
import * as fs from "fs-extra";

export interface Extension {
    readonly name: string;
}

@Injectable({ providedIn: "root" })
export class ExtensionsService implements LoadableService {
    protected extensions: Extension[];

    public constructor(protected readonly workspace: WorkspaceService) {}

    public async load(): Promise<void> {
        this.extensions = await this.fetchExtensions();
    }

    public getExtensions(): Extension[] {
        return this.extensions;
    }

    public getExtensionsWithPrefix(prefix: string): Extension[] {
        const prefixLowerCase = prefix.toLowerCase();

        return this.extensions.filter(extension => {
            const nameWithoutNamespace = extension.name.replace("@emastudio/", '');
            const nameLowerCase = nameWithoutNamespace.toLowerCase();

            return nameLowerCase.startsWith(prefixLowerCase);
        });
    }

    public addExtension(extension: Extension): void {
        this.extensions.push(extension);
    }

    public async getExtensionsFolder(): Promise<string> {
        const rootPath = await this.workspace.getRoot();

        return path.join(rootPath, "extensions");
    }

    protected async getPackageFiles(): Promise<string[]> {
        const relativePattern = path.join("extensions", "*", "package.json");

        return this.workspace.glob(relativePattern);
    }

    public async fetchExtensions(): Promise<Extension[]> {
        const packageFiles = await this.getPackageFiles();
        const extensions = [];

        for (const packageFile of packageFiles) {
            const extensionJSON = await fs.readJSON(packageFile);

            extensions.push(extensionJSON as Extension);
        }

        return extensions;
    }
}
