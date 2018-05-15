/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { ModeQueryHandler } from "./mode-controller";
import URI from "@elysium/core/lib/common/uri";

@injectable()
export class LoadModeQueryHandler implements ModeQueryHandler {
    public canHandle(uri: URI): boolean {
        return uri.getQueryParam("mode") === "load";
    }

    public async handle(uri: URI): Promise<void> {
    }
}
