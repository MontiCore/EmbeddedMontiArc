/* (c) https://github.com/MontiCore/monticore */
import { Platform } from "@services/common/platform";
import { Injectable } from "@angular/core";
import { WorkspaceService } from "@services/common/workspace.service";
import { LoadableService } from "@services/common/load.service";

import * as path from "path";

export interface UseCase {
    readonly name: string;
    readonly platform: Platform;
}

@Injectable({ providedIn: "root" })
export class UseCasesService implements LoadableService {
    protected useCases: UseCase[];

    public constructor(protected readonly workspace: WorkspaceService) {}

    public addUseCase(useCase: UseCase): void {
        this.useCases.push(useCase);
    }

    public async load(): Promise<void> {
        this.useCases = await this.fetchUseCases();
    }

    public getUseCases(): UseCase[] {
        return this.useCases;
    }

    public getUseCasesWithPrefix(prefix: string): UseCase[] {
        const prefixLowerCase = prefix.toLowerCase();

        return this.useCases.filter(useCase => {
            const nameLowerCase = useCase.name.toLowerCase();

            return nameLowerCase.startsWith(prefixLowerCase);
        });
    }

    protected async fetchUseCases(): Promise<UseCase[]> {
        const commonUseCases = await this.getCommonUseCases();
        const windowsUseCases = await this.getWindowsUseCases();
        const linuxUseCases = await this.getLinuxUseCases();

        return [...commonUseCases, ...windowsUseCases, ...linuxUseCases];
    }

    protected async getUseCasesForPlatform(platform: Platform): Promise<UseCase[]> {
        const osFolder = Platform.toFolderName(platform);
        const relativePattern = path.join("target", "src", osFolder, "models", "*");
        const globs = await this.workspace.glob(relativePattern);
        const useCases = [];

        for (const glob of globs) {
            useCases.push({ name: path.basename(glob), platform });
        }

        return useCases;
    }

    protected async getCommonUseCases(): Promise<UseCase[]> {
        return this.getUseCasesForPlatform("all");
    }

    protected async getWindowsUseCases(): Promise<UseCase[]> {
        return this.getUseCasesForPlatform("win32");
    }

    protected async getLinuxUseCases(): Promise<UseCase[]> {
        return this.getUseCasesForPlatform("linux");
    }
}
