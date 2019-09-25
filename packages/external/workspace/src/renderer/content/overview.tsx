/*
 * (c) https://github.com/MontiCore/monticore
 */
import { lazyInject } from "@embeddedmontiarc/sol-external-core/lib/common";
import { DialogService } from "@embeddedmontiarc/sol-external-core/lib/renderer/dialog";
import { bind } from "helpful-decorators";
import { Component, PureComponent, ReactNode } from "react";
import { Workspace, WorkspaceServer } from "../../common";
import { Folder as FolderBase, Delete as DeleteBase } from "@material-ui/icons";
import { Scrollbars } from "react-custom-scrollbars";
import { shell } from "electron";

import {
    CircularProgress, IconButton as IconButtonBase, List as ListBase, ListItem as ListItemBase, ListItemIcon,
    ListItemSecondaryAction, ListItemText, Typography
} from "@material-ui/core";

import * as React from "react";
import * as Path from "path";

import styled from "styled-components";

const color = "color:var(--theia-ui-font-color0);";

const Message = styled.div`${color}`;
const Text = styled(Typography)`text-transform:uppercase; user-select:none;` as any; // tslint:disable-line:no-any
const Folder = styled(FolderBase)`&& { ${color} }` as any; // tslint:disable-line:no-any
const Delete = styled(DeleteBase)`&& { ${color} }` as any; // tslint:disable-line:no-any

const List = styled(ListBase)`
    && .MuiListItem-container {
        background-color:var(--theia-menu-color0);
        ${color}
    }

    && .MuiListItem-container:not(:first-child) {
        margin-top:2px;
    }
`;

const ListItem = styled(ListItemBase)`
    && .MuiListItemText-secondary {
        white-space:nowrap;
        text-overflow:ellipsis;
        overflow:hidden;
        margin-right:20px;
        ${color}
    }
`;

const IconButton = styled(IconButtonBase)`
    && > * {
        opacity:${props => props.disabled ? 0.1 : 1};
    }
`;

/**
 * An interface representing the props passed to the [[OverviewItem]] component.
 */
export interface OverviewItemProps {
    /**
     * Disables the button.
     */
    readonly disabled: boolean;

    /**
     * The workspace which is represented the list item.
     */
    readonly workspace: Workspace;

    /**
     * The event handler to be executed when clicking the list item.
     */
    readonly onClick: () => void;

    /**
     * The event handler to be executed when clicking the trash icon.
     */
    readonly onDeleteClick: () => void;
}

export class OverviewItem extends PureComponent<OverviewItemProps> {
    protected get disabled(): boolean {
        return this.props.disabled;
    }

    protected get workspace(): Workspace {
        return this.props.workspace;
    }

    protected get onClick(): () => void {
        return this.props.onClick;
    }

    protected get onDeleteClick(): () => void {
        return this.props.onDeleteClick;
    }

    protected get path(): string {
        return this.workspace.path;
    }

    protected get basename(): string {
        return Path.basename(this.path);
    }

    public render(): ReactNode {
        return <ListItem button disabled={this.disabled} onClick={this.onClick}>
            <ListItemIcon><Folder/></ListItemIcon>
            <ListItemText primary={this.basename} secondary={this.path}/>
            <ListItemSecondaryAction>{this.renderAction()}</ListItemSecondaryAction>
        </ListItem>;
    }

    protected renderAction(): ReactNode {
        return <IconButton disabled={this.disabled} onClick={this.onDeleteClick}><Delete/></IconButton>;
    }
}

/**
 * An interface representing the props passed to the [[Overview]] component.
 */
export interface OverviewProps {
    /**
     * Disables the button.
     */
    readonly disabled: boolean;
}

/**
 * An interface representing the state of a [[Overview]] component.
 */
export interface OverviewState {
    /**
     * The workspaces to be shown in the list.
     */
    workspaces: Workspace[];

    /**
     * True if the component is loading, false otherwise.
     */
    loading: boolean;
}

export class Overview extends Component<OverviewProps, OverviewState> {
    @lazyInject(DialogService) protected readonly dialogs: DialogService;
    @lazyInject(WorkspaceServer) protected readonly server: WorkspaceServer;

    public constructor(props: OverviewProps) {
        super(props);

        this.state = { workspaces: [], loading: true };

        this.server.getWorkspaces().then(workspaces => this.setState({ workspaces, loading: false }));
    }

    protected get disabled(): boolean {
        return this.props.disabled;
    }

    protected get workspaces(): Workspace[] {
        return this.state.workspaces;
    }

    protected get loading(): boolean {
        return this.state.loading;
    }

    public componentDidMount(): void {
        this.server.on("add", this.onWorkspaceAdd);
        this.server.on("remove", this.onWorkspaceRemove);
    }

    public componentWillUnmount(): void {
        this.server.off("add", this.onWorkspaceAdd);
        this.server.off("remove", this.onWorkspaceRemove);
    }

    public render(): ReactNode {
        if (this.loading) return this.renderProgress();
        else if (this.workspaces.length === 0) return this.renderMessage();
        else return this.renderList();
    }

    protected renderList(): ReactNode {
        return <Scrollbars hideTracksWhenNotNeeded>
            <List dense>{this.renderListItems()}</List>
        </Scrollbars>;
    }

    protected renderListItems(): ReactNode[] {
        return this.workspaces.map(
            workspace => <OverviewItem key={workspace.path} workspace={workspace} disabled={this.disabled}
                                       onClick={() => this.onWorkspaceClick(workspace)}
                                       onDeleteClick={() => this.onWorkspaceDeleteClick(workspace)}
            />
        );
    }

    protected renderMessage(): ReactNode {
        return <Message>
            <Text>Empty</Text>
        </Message>;
    }

    protected renderProgress(): ReactNode {
        return <CircularProgress variant="indeterminate"/>;
    }

    @bind
    protected async onWorkspaceClick(workspace: Workspace): Promise<void> {
        return this.server.open(workspace);
    }

    @bind
    protected async onWorkspaceDeleteClick(workspace: Workspace): Promise<void> {
        const result = await this.dialogs.showMessageBox({
            title: "Workspace Folder",
            message: "The workspace will be deleted. Should the workspace folder itself also be moved to the trash bin?",
            buttons: ["Yes", "No"],
            type: "question"
        });

        if (result.response === 0) shell.moveItemToTrash(workspace.path);

        return this.server.removeWorkspace(workspace.path);
    }

    @bind
    protected onWorkspaceAdd(workspace: Workspace): void {
        this.setState({ workspaces: [...this.workspaces, workspace] });
    }

    @bind
    protected onWorkspaceRemove(path: string): void {
        const index = this.workspaces.findIndex(workspace => workspace.path === path);
        const workspaces = this.workspaces.slice(0);

        if (index > -1) {
            workspaces.splice(index, 1);
            this.setState({ workspaces });
        }
    }
}
