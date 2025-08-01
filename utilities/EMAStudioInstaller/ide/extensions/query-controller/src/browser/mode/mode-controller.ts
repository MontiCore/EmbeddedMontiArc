/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject, named } from "inversify";
import URI from "@elysium/core/lib/common/uri";
import { ContributionProvider } from "@theia/core/lib/common";
import { QueryHandler } from "../query-controller";

export const ModeQueryHandler = Symbol("ModeQueryHandler");

/**
 * `ModeQueryHandler` should be implemented to provide a new query handler for the mode query param.
 */
export interface ModeQueryHandler {
    /**
     * Checks whether the uri can be handled.
     */
    canHandle(uri: URI): boolean;

    /**
     * Handles the uri.
     */
    handle(uri: URI): Promise<void>;
}

@injectable()
export class ModeController implements QueryHandler {
    @inject(ContributionProvider) @named(ModeQueryHandler)
    protected readonly provider: ContributionProvider<ModeQueryHandler>;

    public canHandle(uri: URI): number {
        return uri.hasQueryParam("mode") ? 500 : 0;
    }

    public isChainable(): boolean {
        return false;
    }

    public async handle(uri: URI): Promise<void> {
        for (const contribution of this.provider.getContributions()) {
            if (contribution.canHandle(uri)) return contribution.handle(uri);
        }
    }
}
