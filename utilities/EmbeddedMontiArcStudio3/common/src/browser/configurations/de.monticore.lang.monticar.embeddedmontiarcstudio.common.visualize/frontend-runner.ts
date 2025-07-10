/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationShell } from "@theia/core/lib/browser";
import { MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { MiniBrowserOpenHandler } from "@theia/mini-browser/lib/browser/mini-browser-open-handler";
import { inject, injectable } from "inversify";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";
import { VisualizeFrontendRunnerTOP } from "./frontend-runner-top";

import { VisualizeOptions } from "../../../common/configurations/de.monticore.lang.monticar.embeddedmontiarcstudio.common.visualize/protocol";

import URI from "@theia/core/lib/common/uri";

import WidgetOptions = ApplicationShell.WidgetOptions;

@injectable() // TODO: It would be better to create a new instance instead of using a singleton for configuration runners.
export class VisualizeFrontendRunner extends VisualizeFrontendRunnerTOP {
    @inject(MiniBrowserOpenHandler) openHandler: MiniBrowserOpenHandler;

    protected async runOpen(uuid: string, options: VisualizeOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        const workspace = context.workspace && new URI(context.workspace);
        const visualization = workspace && workspace.resolve(`${options.outputPath}/index.html`);
        const props = <MiniBrowserProps>{ toolbar: "read-only", name: "Visualization" };
        const widgetOptions = <WidgetOptions>{ area: "main", mode: "split-bottom" };
        const openerOptions = { ...props, widgetOptions };

        console.log(`options.outputPath=${options.outputPath}`);

        if (visualization) await this.openHandler.open(visualization, openerOptions);
    }
}
