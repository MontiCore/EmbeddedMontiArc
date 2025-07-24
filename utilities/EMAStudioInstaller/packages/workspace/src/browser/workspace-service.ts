/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { WorkspaceService as BaseWorkspaceService } from "@theia/workspace/lib/browser";

@injectable()
export class WorkspaceService extends BaseWorkspaceService {
    protected updateTitle(): void {
        const titles = document.getElementsByTagName("title");
        const title = titles[0].innerText;

        if (this.workspace) super.updateTitle();
        else document.title = title;
    }
}
