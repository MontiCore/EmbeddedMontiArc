/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Application } from "@embeddedmontiarc/sol-external-core/lib/main";
import { inject, injectable } from "inversify";
import { DockerRunContribution, DockerRunRegistry } from "./docker-run-registry";

import * as path from "path";
import * as os from "os";
import * as fs from "fs-extra";

export const DockerVolumeService = Symbol("DockerVolumeService");
/**
 * An interface to be implemented by classes which operate on the volumes of Docker.
 */
export interface DockerVolumeService {

}

@injectable()
export class DockerVolumeServiceImpl implements DockerVolumeService, DockerRunContribution {
    @inject(Application) protected readonly application: Application;

    public registerArguments(registry: DockerRunRegistry): void {
        registry.registerArgument({
            for: "run",
            resolve: (hostPath: string) => this.resolveMount(hostPath)
        });

        registry.registerArgument({
            for: "run",
            resolve: () => this.resolveTheiaDirectory()
        });

        registry.registerArgument({
            for: "run",
            resolve: () => this.resolvePluginsDirectory()
        });
    }

    protected async resolveMount(hostPath: string): Promise<string> {
        const unixHostPath = this.convertToUnixPath(hostPath);

        return `-v "${unixHostPath}:/home/workspace"`;
    }

    protected async resolveTheiaDirectory(): Promise<string> {
        const theiaDirectory = path.resolve(os.homedir(), ".theia");

        await fs.ensureDir(theiaDirectory);

        const unixTheiaDirectory = this.convertToUnixPath(theiaDirectory);

        return `-v "${unixTheiaDirectory}:/root/.theia"`;
    }

    protected async resolvePluginsDirectory(): Promise<string> {
        const applicationName = this.application.getName().toLowerCase();
        const pluginsDirectory = path.resolve(os.homedir(), `.${applicationName}`, "plugins");

        await fs.ensureDir(pluginsDirectory);

        const unixPluginsDirectory = this.convertToUnixPath(pluginsDirectory);

        return `-v "${unixPluginsDirectory}:/home/plugins"`;
    }

    protected convertToUnixPath(hostPath: string): string {
        const result = path.parse(hostPath);

        if (result.root === '/') return hostPath;

        const root = hostPath.substr(0, 1).toLowerCase();
        const rest = hostPath.substr(3).replace(/\\/g, '/');

        return `/${root}/${rest}`;
    }
}
