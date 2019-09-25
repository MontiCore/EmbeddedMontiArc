/* (c) https://github.com/MontiCore/monticore */

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

        logo.id = "sol:se-logo";
        logo.addClass("sol-se-logo");
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
