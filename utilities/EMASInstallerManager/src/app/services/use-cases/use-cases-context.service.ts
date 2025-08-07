/* (c) https://github.com/MontiCore/monticore */
import { UseCaseState } from "@services/use-cases/use-cases-state.service";
import { Injectable } from "@angular/core";

import * as path from "path";

export class UseCaseContext {
    protected readonly state: UseCaseState;

    public constructor(state: UseCaseState) {
        this.state = state;
    }

    public get name(): string {
        return this.state.name;
    }

    protected get folder(): File {
        return this.state.folder!;
    }

    protected get mainFile(): File | undefined {
        return this.state.mainFile;
    }

    public hasMainFile(): boolean {
        return this.mainFile !== undefined;
    }

    public getMainFileWorkspacePath(): string {
        const modelRoot = this.folder.path;
        const mainFile = this.mainFile!.path;
        const relativePath = path.relative(modelRoot, mainFile);

        return relativePath.replace(/\\/g, '/');
    }
}

@Injectable({ providedIn: "root" })
export class UseCasesContextService {
    public createContext(state: UseCaseState): UseCaseContext {
        return new UseCaseContext(state);
    }
}
