/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { WindowService, DefaultWindowService } from "@theia/core/lib/browser/window/window-service";

export const ExtendedWindowService = Symbol("ExtendedWindowService");

export interface ExtendedWindowService extends WindowService {
    /**
     * Redirects the current window to the given URL.
     */
    redirectWindow(url: string): void;
}

@injectable()
export class DefaultExtendedWindowService extends DefaultWindowService implements ExtendedWindowService {
    public redirectWindow(url: string): void {
        window.location.href = url;
    }
}
