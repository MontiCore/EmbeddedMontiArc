/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
import { MaybePromise } from "@theia/core/lib/common";
import { LabelProviderContribution } from "@theia/core/lib/browser";
import URI from "@theia/core/lib/common/uri";
import { inject, injectable } from "inversify";
import { ModulesService } from "./modules-service";

@injectable()
export class ModulesLabelProvider implements LabelProviderContribution {
    @inject(ModulesService) protected readonly modules: ModulesService;

    public canHandle(element: object): number {
        return 0;
    }

    public getIcon(element: URI): MaybePromise<string> {
        return "";
    }

    public getName(element: URI): string {
        return "";
    }

    public getLongName(element: URI): string {
        return "";
    }
}
