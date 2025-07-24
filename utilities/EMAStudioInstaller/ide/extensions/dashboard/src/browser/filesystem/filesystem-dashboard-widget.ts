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

const CONTROL_ITEM_CLASS = "elysium-dashboard-item elysium-dashboard-icon-plus";
const DASHBOARD_BUSY_CLASS = "elysium-dashboard-busy";

@injectable()
export class FileSystemDashboardWidget extends BaseDashboardWidget {
    @inject(DashboardProps) public readonly props: DashboardProps;
    @inject(FileSystemDashboardModel) public readonly model: FileSystemDashboardModel;
    @inject(ContextMenuRenderer) public readonly contextMenuRenderer: ContextMenuRenderer;
    @inject(FileSystemDashboardQueue) public readonly queue: FileSystemDashboardQueue;

    @inject(DemosDashboardWidget) protected readonly demosWidget: DemosDashboardWidget;

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

    protected async handleItemClickEvent(event: MouseEvent, item: DashboardItem): Promise<void> {
        await super.handleItemClickEvent(event, item);
        this.model.selectItem(<Readonly<SelectableDashboardItem>>item);
        if (this.props.itemContextMenu && !this.queue.isBusy()) this.contextMenuRenderer.render(this.props.itemContextMenu, event);
    }

    protected async handleItemMouseOverEvent(event: MouseEvent, item: DashboardItem): Promise<void> {
        await super.handleItemMouseOverEvent(event, item);

        const element = event.toElement;

        if (this.queue.isBusy()) element.classList.add(DASHBOARD_BUSY_CLASS);
        else element.classList.remove(DASHBOARD_BUSY_CLASS);
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
