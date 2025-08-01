/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";
import { UseCasesStateService, UseCaseState } from "@services/use-cases/use-cases-state.service";
import { WorkspaceService } from "@services/common/workspace.service";
import { Platform } from "@services/common/platform";
import { UseCaseGeneratorService } from "@services/use-cases/use-case-generator.service";
import { UseCase, UseCasesService } from "@services/use-cases/use-cases.service";

import * as path from "path";
import * as fs from "fs-extra";

export class UseCaseStateTransformation {
    protected readonly state: UseCaseState;
    protected readonly workspace: WorkspaceService;
    protected readonly generator: UseCaseGeneratorService;

    public constructor(state: UseCaseState, workspace: WorkspaceService, generator: UseCaseGeneratorService) {
        this.state = state;
        this.workspace = workspace;
        this.generator = generator;
    }

    protected get folder(): File {
        return this.state.folder!;
    }

    protected async getDirectory(): Promise<string> {
        const rootPath = await this.workspace.getRoot();
        const platformFolder = Platform.toFolderName(this.state.platform);

        return path.join(rootPath!, "target", "src", platformFolder, "models", this.state.name);
    }

    public async apply(): Promise<UseCase> {
        const directory = await this.getDirectory();

        await fs.ensureDir(directory);
        await fs.copy(this.folder.path, directory);
        await this.generator.generate(directory, this.state);

        return { name: this.state.name, platform: this.state.platform } as UseCase;
    }
}

@Injectable({ providedIn: "root" })
export class UseCasesStateTransformerService {
    public constructor(
        protected readonly workspace: WorkspaceService,
        protected readonly generator: UseCaseGeneratorService,
        protected readonly states: UseCasesStateService,
        protected readonly collection: UseCasesService
    ) {}

    public async apply(): Promise<void> {
        const states = this.states.getStates(true) as UseCaseState[];

        for (const state of states) {
            const transformation = new UseCaseStateTransformation(state, this.workspace, this.generator);
            const useCase = await transformation.apply();

            state.saved = true;

            this.collection.addUseCase(useCase);
        }
    }
}
