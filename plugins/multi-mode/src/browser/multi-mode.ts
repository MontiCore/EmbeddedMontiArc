/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject, named } from "inversify";
import URI from "@elysium/core/lib/common/uri";
import { ContributionProvider, DisposableCollection } from "@theia/core/lib/common";
import { FrontendApplicationContribution, FrontendApplication } from "@theia/core/lib/browser";
import { FrontendApplicationStateService } from "@theia/core/lib/browser/frontend-application-state";
import { FrontendApplicationState } from "@theia/core/lib/browser/frontend-application-state";

export const MultiModeContribution = Symbol("MultiModeContribution");

export interface MultiModeContribution {
    /**
     * The mode that can be handled by the contribution.
     */
    readonly mode: string;

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
export abstract class BaseMultiModeContribution implements MultiModeContribution {
    public abstract readonly mode: string;

    public canHandle(uri: URI): boolean {
        return uri.getQueryParam("mode") === this.mode;
    }

    public abstract async handle(uri: URI): Promise<void>;
}

@injectable()
export class MultiModeProcessor implements FrontendApplicationContribution {
    @inject(ContributionProvider) @named(MultiModeContribution)
    protected readonly provider: ContributionProvider<MultiModeContribution>;
    @inject(FrontendApplicationStateService) protected readonly stateService: FrontendApplicationStateService;

    protected toDipose: DisposableCollection = new DisposableCollection();

    public async onStart(app: FrontendApplication): Promise<void> {
        this.toDipose.push(
            this.stateService.onStateChanged(
                async (state: FrontendApplicationState) => await this.handleStateChange(state)
            )
        );
    }

    protected async handleStateChange(state: FrontendApplicationState): Promise<void> {
        if (state === "started_contributions") {
            this.toDipose.dispose();

            return this.handleContributionsStarted();
        }
    }

    protected async handleContributionsStarted(): Promise<void> {
        const uri = new URI(window.location.href);

        for (const contribution of this.provider.getContributions()) {
            if (contribution.canHandle(uri)) return contribution.handle(uri);
        }
    }
}
