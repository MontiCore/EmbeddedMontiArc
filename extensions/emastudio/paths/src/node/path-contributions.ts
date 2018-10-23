/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { PathContribution, PathsRegistry } from "./paths-registry";
import { FileUri } from "@theia/core/lib/node/file-uri";
import { CliContribution } from "@theia/core/lib/node";

import * as Yargs from "yargs";

import ArgumentVector = Yargs.Argv;
import Arguments = Yargs.Arguments;

@injectable()
export class RootPathContribution implements PathContribution, CliContribution {
    protected resourcesPath: string;

    public registerTo(registry: PathsRegistry): void {
        const resourcesPath = this.resourcesPath.replace(/\\/g, '/');

        registry.setPath('.', FileUri.create(resourcesPath));
    }

    public configure(configuration: ArgumentVector): void {
        configuration.option("resources-path", {
            description: "Path to resources folder.",
            type: "string",
            nargs: 1
        });
    }

    public async setArguments(args: Arguments): Promise<void> {
        this.resourcesPath = args.resourcesPath;
    }
}
