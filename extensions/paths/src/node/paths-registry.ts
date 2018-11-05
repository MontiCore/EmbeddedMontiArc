/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import URI from "@theia/core/lib/common/uri";
import { inject, injectable, named } from "inversify";
import { BackendApplicationContribution } from "@theia/core/lib/node/backend-application";
import { ContributionProvider } from "@theia/core/lib/common";

export const PathContribution = Symbol("PathContribution");

/**
 * `PathContribution` should be implemented to provide a new path contribution.
 */
export interface PathContribution {
    registerTo(registry: PathsRegistry): void;
}

@injectable()
export class PathsRegistry implements BackendApplicationContribution {
    @inject(ContributionProvider) @named(PathContribution)
    protected readonly provider: ContributionProvider<PathContribution>;

    protected readonly paths: Map<string, URI>;

    public constructor() {
        this.paths = new Map<string, URI>();
    }

    public getRoot(): URI {
        return this.paths.get('.')!;
    }

    public setPath(id: string, uri: URI): void {
        this.paths.set(id, uri);
    }

    public getPath(id: string): URI {
        return this.paths.get(id)!;
    }

    public initialize() {
        for (const contribution of this.provider.getContributions()) {
            contribution.registerTo(this);
        }
    }
}
