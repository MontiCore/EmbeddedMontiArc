/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationShell } from "@theia/core/lib/browser";
import { MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { MiniBrowserOpenHandler } from "@theia/mini-browser/lib/browser/mini-browser-open-handler";
import { inject, injectable } from "inversify";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";
import { ClusterFiddleExecuteFrontendRunnerTOP } from "./frontend-runner-top";

import { ClusterFiddleExecuteOptions } from "../../../common/configurations/de.monticore.lang.monticar.embeddedmontiarcstudio.clusterfiddle.clusterfiddleexecute/protocol";

import URI from "@theia/core/lib/common/uri";

import WidgetOptions = ApplicationShell.WidgetOptions;

@injectable()
export class ClusterFiddleExecuteFrontendRunner extends ClusterFiddleExecuteFrontendRunnerTOP {
    @inject(MiniBrowserOpenHandler) openHandler: MiniBrowserOpenHandler;

    protected async runOpen(uuid: string, options: ClusterFiddleExecuteOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        const workspace = context.workspace && new URI(context.workspace);
        const clusterFiddle = workspace && workspace.resolve(`${options.outputDirectory}/clusterfiddle/index.html`);
        const props = <MiniBrowserProps>{ toolbar: "read-only", name: "ClusterFiddle" };
        const widgetOptions = <WidgetOptions>{ area: "main", mode: "tab-after" };
        const openerOptions = { ...props, widgetOptions };

        console.log(`options.outputDirectory=${options.outputDirectory}`);

        if (clusterFiddle) await this.openHandler.open(clusterFiddle, openerOptions);
    }
}
