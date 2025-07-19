/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject, named } from "inversify";
import { BackendApplicationContribution } from "@theia/core/lib/node/backend-application";
import URI from "@theia/core/lib/common/uri";
import { ContributionProvider } from "@theia/core/lib/common";
import * as Express from "express";
import { FileUri } from "@theia/core/lib/node/file-uri";

import Application = Express.Application;

export const StaticContribution = Symbol("StaticContribution");

export interface StaticContribution {
    readonly path: string;
    readonly uri: URI;
}

@injectable()
export class StaticRegistry implements BackendApplicationContribution {
    @inject(ContributionProvider) @named(StaticContribution)
    protected readonly provider: ContributionProvider<StaticContribution>;

    public configure(application: Application): void {
        for (const contribution of this.provider.getContributions()) {
            const uri = FileUri.fsPath(contribution.uri);

            application.use(contribution.path, Express.static(uri));
        }
    }
}
