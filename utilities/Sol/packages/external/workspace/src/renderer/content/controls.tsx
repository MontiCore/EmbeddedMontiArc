/*
 * (c) https://github.com/MontiCore/monticore
 */
import { lazyInject } from "@embeddedmontiarc/sol-external-core/lib/common";
import { DialogService } from "@embeddedmontiarc/sol-external-core/lib/renderer/dialog/dialog-service";
import { ListItem as ListItemBase, Tooltip } from "@material-ui/core";
import { Add, SvgIconComponent } from "@material-ui/icons";
import { bind } from "helpful-decorators";
import { Component, PureComponent, ReactNode } from "react";
import { Git, Import } from "mdi-material-ui";
import { OpenDialogOptions } from "electron";
import { WorkspaceServer } from "../../common";

import * as React from "react";

import styled from "styled-components";

const Container = styled.div`
    && {
        display:flex;
        justify-content:center;
        align-items:center;
        height:100%;
        flex-direction:row;
    }
`;

const ListItem = styled(ListItemBase)`
    && {
        width:100px;
        height:100px;
        color:var(--theia-ui-font-color0);
        background-color:var(--theia-menu-color0);
        display:flex;
        justify-content:center;
        align-items:center;
    }

    &&:not(:first-child) {
        margin-left:15px;
    }
`;

/**
 * An interface representing the props passed to a [[Control]] component.
 */
export interface ControlProps {
    /**
     * The label of the tooltip shown when hovering the button.
     */
    readonly label: string;

    /**
     * The icon to be shown on the button.
     */
    readonly icon: SvgIconComponent;

    /**
     * Disables the button.
     */
    readonly disabled?: boolean;

    /**
     * An event handler to be executed when clicking the button.
     */
    readonly onClick: () => void;
}

export class Control extends PureComponent<ControlProps> {
    protected get label(): string {
        return this.props.label;
    }

    protected get icon(): SvgIconComponent {
        return this.props.icon;
    }

    protected get disabled(): boolean {
        return this.props.disabled || false;
    }

    protected get onClick(): () => void {
        return this.props.onClick;
    }

    public render(): ReactNode {
        return <Tooltip title={this.label}>
            <ListItem button onClick={this.onClick} disabled={this.disabled}>
                {this.renderIcon()}
            </ListItem>
        </Tooltip>;
    }

    protected renderIcon(): ReactNode {
        const Icon = this.icon;
        return <Icon fontSize="large"/>;
    }
}

/**
 * An interface representing the props passed to the [[Controls]] component.
 */
export interface ControlsProps {
    /**
     * Disables the button.
     */
    readonly disabled: boolean;
}

export class Controls extends Component<ControlsProps> {
    @lazyInject(DialogService) protected readonly dialogs: DialogService;
    @lazyInject(WorkspaceServer) protected readonly workspaces: WorkspaceServer;

    protected get disabled(): boolean {
        return this.props.disabled;
    }

    public render(): ReactNode {
        return <Container>
            <Control label="Add Workspace" icon={Add} onClick={this.onAddClick} disabled/>
            <Control label="Import Source Code" icon={Import} onClick={this.onImportClick} disabled={this.disabled}/>
            <Control label="Add From Version Control" icon={Git} onClick={this.onAddVCSClick} disabled/>
        </Container>;
    }

    @bind
    protected async onAddClick(): Promise<void> {}

    @bind
    protected async onImportClick(): Promise<void> {
        const options: OpenDialogOptions = { title: "Choose Workspace", properties: ["openDirectory"] };
        const result = await this.dialogs.showOpenDialog(options);
        const filePaths = result.filePaths;

        if (filePaths && filePaths.length > 0) return this.workspaces.addWorkspace({ path: filePaths[0] });
    }

    @bind
    protected onAddVCSClick(): void {}
}
