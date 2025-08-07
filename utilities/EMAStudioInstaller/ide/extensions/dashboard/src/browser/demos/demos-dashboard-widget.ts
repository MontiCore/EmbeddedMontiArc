/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { BaseDashboardWidget, DashboardProps } from "../dashboard-widget";
import { DemosDashboardModel } from "./demos-dashboard-model";
import { ContextMenuRenderer } from "@theia/core/lib/browser";
import { h } from "@phosphor/virtualdom";
import { DashboardItem } from "../dashboard";
import { Widget } from "@theia/core/lib/browser";
import { AsyncSingleTextInputDialog } from "@elysium/core/lib/browser";
import { FileSystem } from "@theia/filesystem/lib/common";
import { FileUri } from "@theia/core/lib/node/file-uri";

export const CONTROL_ITEM_CLASS = "elysium-dashboard-item elysium-dashboard-icon-back";

@injectable()
export class DemosDashboardWidget extends BaseDashboardWidget {
    @inject(DashboardProps) public readonly props: DashboardProps;
    @inject(DemosDashboardModel) public readonly model: DemosDashboardModel;
    @inject(ContextMenuRenderer) public readonly contextMenuRenderer: ContextMenuRenderer;
    @inject(FileSystem) protected readonly fileSystem: FileSystem;

    protected renderControlItem(): h.Child {
        const options = {
            className: CONTROL_ITEM_CLASS,
            onclick: async (event: MouseEvent) => await this.handleControlEvent(event)
        };

        return h.li(options);
    }

    protected async handleControlEvent(event: MouseEvent): Promise<void> {
        event.stopPropagation();
        Widget.detach(this);
    }

    protected async handleItemClickEvent(event: MouseEvent, item: DashboardItem): Promise<void> {
        await super.handleItemClickEvent(event, item);

        const options = {
            title: "Workspace Name",
            initialValue: "Workspace-Name",
            validateAsync: this.validateWorkspaceName.bind(this)
        };
        const dialog = new AsyncSingleTextInputDialog(options);
        const target = await dialog.open();
        const uri = FileUri.create(`/${target}`).toString();

        Widget.detach(this);
        return this.model.clone(uri, item);
    }

    protected async validateWorkspaceName(name: string): Promise<string> {
        const uri = FileUri.fsPath(`/${name}`).toString();

        if (name.length === 0) return "Please enter a workspace name";
        if (name.match(/[^\w\-.]+/)) return "The workspace name contains illegal characters";
        if (await this.fileSystem.exists(uri)) return "A workspace with such a name already exists";

        return '';
    }
}
