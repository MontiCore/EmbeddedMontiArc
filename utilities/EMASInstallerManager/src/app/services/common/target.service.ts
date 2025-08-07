/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";
import { WorkspaceService } from "@services/common/workspace.service";
import { Extension } from "@services/extensions/extensions.service";
import { LoadableService } from "@services/common/load.service";

import * as path from "path";
import * as fs from "fs-extra";

export interface Target {
    readonly dependencies: { [key: string]: string };
}

@Injectable({ providedIn: "root" })
export class TargetService implements LoadableService {
    protected target: Target;

    public constructor(protected readonly workspace: WorkspaceService) {}

    public async load(): Promise<void> {
        this.target = await this.fetchTarget();
    }

    public async getPackageFile(): Promise<string> {
        const relativePattern = path.join("target", "package.json");
        const dependenciesFiles = await this.workspace.glob(relativePattern);

        return dependenciesFiles[0];
    }

    protected async fetchTarget(): Promise<Target> {
        const packageFile = await this.getPackageFile();
        const packageJSON = await fs.readJSON(packageFile);

        return packageJSON as Target;
    }

    public addDependency(dependency: Extension): void {
        this.target.dependencies[dependency.name] = "^0.1.0";
    }
}
