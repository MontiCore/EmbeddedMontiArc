/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";
import { WorkspaceService } from "@services/common/workspace.service";
import { LoadableService } from "@services/common/load.service";
import { Platform } from "@services/common/platform";

import * as path from "path";
import * as fs from "fs-extra";

export interface Artifact {
    readonly comment: string;
    readonly platforms: Platform[];
    readonly from: string;
    readonly to: string;
}

@Injectable({ providedIn: "root" })
export class ArtifactsService implements LoadableService {
    protected artifacts: Artifact[];

    public constructor(protected readonly workspace: WorkspaceService) {}

    public async load(): Promise<void> {
        this.artifacts = await this.fetchArtifacts();
    }

    public getArtifacts(copy: boolean = false): Artifact[] {
        return copy ? this.artifacts.slice(0) : this.artifacts;
    }

    public addArtifact(artifact: Artifact): void {
        this.artifacts.push(artifact);
    }

    public async getDependenciesFile(): Promise<string> {
        const relativePattern = path.join("configs", "**", "dependencies.json");
        const dependenciesFiles = await this.workspace.glob(relativePattern);

        return dependenciesFiles[0];
    }

    protected async fetchArtifacts(): Promise<Artifact[]> {
        const dependenciesFile = await this.getDependenciesFile();
        const dependencies = await fs.readJSON(dependenciesFile);

        return dependencies as Artifact[];
    }
}
