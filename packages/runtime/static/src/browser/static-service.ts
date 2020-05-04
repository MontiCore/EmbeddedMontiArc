/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationShell, WidgetManager, Endpoint } from "@theia/core/lib/browser";
import { inject, injectable } from "inversify";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";

import WidgetOptions = ApplicationShell.WidgetOptions;

export const StaticService = Symbol("StaticService");
export interface StaticService {
    open(path: string): Promise<void>;
}

@injectable()
export class StaticServiceImpl implements StaticService {
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;

    public async open(path: string): Promise<void> {
        const endpoint = new Endpoint({ path: `/workspace/${encodeURIComponent(path)}` });
        const startPageUri = endpoint.getRestUrl();
        const widgetOptions = { uri: startPageUri };
        const props = <MiniBrowserProps>{ startPage: startPageUri.toString(), toolbar: "read-only", name: "Visualization" };
        const widget = await this.widgetManager.getOrCreateWidget<MiniBrowser>(MiniBrowser.ID, widgetOptions);
        const addOptions = <WidgetOptions>{ area: "main", mode: "split-right" };

        widget.setProps(props);
        widget.update();

        if (widget.isAttached) this.shell.revealWidget(widget.id);
        else return this.shell.addWidget(widget, addOptions);
    }
}
