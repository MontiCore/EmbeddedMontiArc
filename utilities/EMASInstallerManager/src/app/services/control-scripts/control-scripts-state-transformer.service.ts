/* (c) https://github.com/MontiCore/monticore */
import { ControlScriptsStateService, ControlScriptState } from "@services/control-scripts/control-scripts-state.service";
import { ControlScript } from "@services/control-scripts/control-scripts.service";
import { WorkspaceService } from "@services/common/workspace.service";
import { Platform } from "@services/common/platform";

import * as path from "path";
import * as fs from "fs-extra";
import { Injectable } from "@angular/core";

export class ControlScriptStateTransformation {
    protected readonly state: ControlScriptState;
    protected readonly workspace: WorkspaceService;

    public constructor(state: ControlScriptState, workspace: WorkspaceService) {
        this.state = state;
        this.workspace = workspace;
    }

    protected get file(): File {
        return this.state.path!;
    }

    protected async getBaseDirectory(): Promise<string> {
        const rootPath = await this.workspace.getRoot();
        const platform = this.file.path.endsWith(".bat") ? "win32" : "linux";
        const platformFolder = Platform.toFolderName(platform);

        return path.join(rootPath, "target", "src", platformFolder, "scripts");
    }

    protected async getDestination(): Promise<string> {
        const base = await this.getBaseDirectory();

        return path.join(base, this.state.useCase, this.state.extension.toLowerCase(), this.state.name);
    }

    public async apply(): Promise<ControlScript> {
        const destination = await this.getDestination();

        await fs.copy(this.file.path, destination);

        return { name: this.state.name, useCase: this.state.useCase, extension: this.state.extension } as ControlScript;
    }
}

@Injectable({ providedIn: "root" })
export class ControlScriptsStateTransformerService {
    public constructor(
        protected readonly workspace: WorkspaceService,
        protected readonly states: ControlScriptsStateService
    ) {}

    public async apply(): Promise<void> {
        const states = this.states.getStates(true) as ControlScriptState[];

        for (const state of states) {
            const transformation = new ControlScriptStateTransformation(state, this.workspace);
            /*const script = */await transformation.apply();

            state.saved = true;

            // TODO: Add script to collection.
        }
    }
}
