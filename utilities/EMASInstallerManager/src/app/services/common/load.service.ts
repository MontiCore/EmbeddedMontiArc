/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";
import { ArtifactsService } from "@services/artifacts";
import { UseCasesService } from "@services/use-cases";
import { TargetService } from "@services/common/target.service";
import { ExtensionsService } from "@services/extensions/extensions.service";

export interface LoadableService {
    load(): Promise<void>;
}

@Injectable({ providedIn: "root" })
export class LoadService implements LoadableService {
    public constructor(
        protected readonly artifacts: ArtifactsService,
        protected readonly useCases: UseCasesService,
        protected readonly target: TargetService,
        protected readonly extensions: ExtensionsService
    ) {}

    public async load(): Promise<void> {
        await Promise.all([
            this.artifacts.load(), this.useCases.load(),
            this.target.load(), this.extensions.load()
        ]);
    }
}
