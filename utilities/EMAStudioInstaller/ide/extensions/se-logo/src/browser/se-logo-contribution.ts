/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { FrontendApplicationContribution, FrontendApplication } from "@theia/core/lib/browser";
import { Widget } from "@phosphor/widgets";
import { WindowService } from "@theia/core/lib/browser/window/window-service";

@injectable()
export class SELogoContribution implements FrontendApplicationContribution {
    @inject(WindowService)
    protected readonly windowService: WindowService;

    public onStart(app: FrontendApplication): void {
        const logo = new Widget();

        logo.id = "elysium:se-logo";
        logo.addClass("elysium-se-logo");
        logo.node.addEventListener(
            "click",
            () => this.handleClick()
        );
        app.shell.addWidget(logo, { area: "top", mode: "tab-before" });
    }

    protected handleClick() {
        this.windowService.openNewWindow("http://www.se-rwth.de");
    }
}
