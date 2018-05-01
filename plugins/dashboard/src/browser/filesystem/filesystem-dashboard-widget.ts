/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable, postConstruct } from "inversify";
import { BaseDashboardWidget, DashboardProps } from "../dashboard-widget";
import { FileSystemDashboardModel } from "./filesystem-dashboard-model";
import { ContextMenuRenderer } from "@theia/core/lib/browser";
import { DashboardItem } from "../dashboard";
import { h } from "@phosphor/virtualdom";
import { SelectableDashboardItem } from "../dashboard-selection";
import { DemosDashboardWidget } from "../demos/demos-dashboard-widget";
import { FileSystemDashboardQueue } from "./filesystem-dashboard-queue";

export const CONTROL_ITEM_CLASS = "elysium-dashboard-item elysium-dashboard-icon-plus";

@injectable()
export class FileSystemDashboardWidget extends BaseDashboardWidget {
    @inject(DashboardProps) public readonly props: DashboardProps;
    @inject(FileSystemDashboardModel) public readonly model: FileSystemDashboardModel;
    @inject(ContextMenuRenderer) public readonly contextMenuRenderer: ContextMenuRenderer;

    @inject(DemosDashboardWidget) protected readonly demosWidget: DemosDashboardWidget;
    @inject(FileSystemDashboardQueue) protected readonly queue: FileSystemDashboardQueue;

    @postConstruct()
    protected init(): void {
        super.init();

        this.demosWidget.model.onClone(async uri => this.handleCloneEvent(uri));
        this.demosWidget.model.onCloned(async uri => this.handleClonedEvent(uri));
    }

    protected renderControlItem(): h.Child {
        const options = {
            className: CONTROL_ITEM_CLASS,
            onclick: async (event: MouseEvent) => await this.handleControlEvent(event)
        };

        return h.li(options);
    }

    protected async handleControlEvent(event: MouseEvent): Promise<void> {
        event.stopPropagation();
        if (this.props.controlContextMenu) this.contextMenuRenderer.render(this.props.controlContextMenu, event);
    }

    protected async handleItemEvent(event: MouseEvent, item: DashboardItem): Promise<void> {
        await super.handleItemEvent(event, item);
        this.model.selectItem(<Readonly<SelectableDashboardItem>>item);
        if (this.props.itemContextMenu) this.contextMenuRenderer.render(this.props.itemContextMenu, event);
    }

    protected async handleCloneEvent(uri: string): Promise<void> {
        this.queue.addToQueue(uri);
        await this.model.refresh();
    }

    protected async handleClonedEvent(uri: string): Promise<void> {
        this.queue.removeFromQueue(uri);
        await this.model.refresh();
    }
}
