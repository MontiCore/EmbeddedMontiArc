/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { screen } from "electron";

import {
    DockerRunContribution,
    DockerRunRegistry
} from "@embeddedmontiarc/sol-external-docker/lib/main/docker-run-registry";

@injectable()
export class GUIProcessRunContribution implements DockerRunContribution {
    public registerArguments(registry: DockerRunRegistry): void {
        registry.registerArgument({
            for: "image",
            resolve: () => this.resolveResolution()
        });
    }

    protected async resolveResolution(): Promise<string> {
        const display = screen.getPrimaryDisplay();
        const size = display.size;

        return `--resolution=${size.width}x${size.height}`;
    }
}
