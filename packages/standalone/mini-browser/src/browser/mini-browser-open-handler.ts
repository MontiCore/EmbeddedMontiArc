/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { MiniBrowserOpenHandler as BaseMiniBrowserOpenHandler } from "@theia/mini-browser/lib/browser/mini-browser-open-handler";
import URI from "@theia/core/lib/common/uri";

@injectable()
export class MiniBrowserOpenHandler extends BaseMiniBrowserOpenHandler {
    public canHandle(uri: URI): number {
        return uri.scheme.startsWith("http") ? 1 : 0;
    }
}
