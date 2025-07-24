/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject, named } from "inversify";
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { ContributionProvider } from "@theia/core/lib/common";

export const AccessContribution = Symbol("AccessContribution");

/**
 * `AccessContribution` should be implemented to provide a new access contribution.
 */
export interface AccessContribution {
    /**
     * The identifier under which the API will be registered.
     */
    readonly id: string;

    /**
     * The API for external or internal use.
     */
    readonly contribution: object;
}

/**
 * `FrontendApplicationContribution` responsible for the registration of all access contributions.
 */
@injectable()
export class AccessController implements FrontendApplicationContribution {
    @inject(ContributionProvider) @named(AccessContribution)
    protected readonly provider: ContributionProvider<AccessContribution>;

    // tslint:disable:no-any
    public initialize(): void {
        (window as any).api = {};

        for (const contribution of this.provider.getContributions()) {
            (window as any).api[contribution.id] = contribution.contribution;
        }
    }
    // tslint:enable:no-any
}
