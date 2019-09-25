/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { DockerRunContribution, DockerRunRegistry } from "./docker-run-registry";

import * as path from "path";

export const DockerVolumeService = Symbol("DockerVolumeService");
/**
 * An interface to be implemented by classes which operate on the volumes of Docker.
 */
export interface DockerVolumeService {

}

@injectable()
export class DockerVolumeServiceImpl implements DockerVolumeService, DockerRunContribution {
    public registerArguments(registry: DockerRunRegistry): void {
        registry.registerArgument({
            for: "run",
            resolve: (hostPath: string) => this.resolveMount(hostPath)
        });
    }

    protected async resolveMount(hostPath: string): Promise<string> {
        const unixHostPath = this.convertToUnixPath(hostPath);

        return `-v "${unixHostPath}:/home/project"`;
    }

    protected convertToUnixPath(hostPath: string): string {
        const result = path.parse(hostPath);

        if (result.root === '/') return hostPath;

        const root = hostPath.substr(0, 1).toLowerCase();
        const rest = hostPath.substr(3).replace(/\\/g, '/');

        return `/${root}/${rest}`;
    }
}
