/*
 * (c) https://github.com/MontiCore/monticore
 */
import { FileDialogService, OpenFileDialogProps } from "@theia/filesystem/lib/browser";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { bind } from "helpful-decorators";
import { interfaces } from "inversify";
import { OptionProps, ReturnsOption } from "./option";

import URI from "@theia/core/lib/common/uri";

import * as React from "react";

import ReactNode = React.ReactNode;
import Container = interfaces.Container;

/**
 * Represents the props passed to a path component.
 */
export interface PathProps extends OptionProps<string> {
    readonly label: string;
    readonly required: boolean;
    readonly workspace: WorkspaceService;
    readonly fileDialog: FileDialogService;
}

/**
 * Implementation of a path component.
 */
export class PathOption extends ReturnsOption<string, PathProps> {
    protected customError: string | undefined;

    public constructor(props: PathProps) {
        super(props, '');
    }

    protected get workspace(): WorkspaceService {
        return this.props.workspace;
    }

    protected get fileDialog(): FileDialogService {
        return this.props.fileDialog;
    }

    protected get label(): string {
        return `${this.props.label}${this.props.required ? '*' : ''}`;
    }

    protected get className(): string {
        return `sol-runtime-options component ${this.error ? "invalid" : ''} path-option`;
    }

    public render(): ReactNode {
        return <div className={this.className}>
            {this.renderValue()}
            {this.renderError()}
        </div>;
    }

    protected renderValue(): ReactNode {
        return <div className="value">
            <span className="label">{this.label}</span>
            <input type="text" value={this.value} readOnly={true}/>
            <button className="theia-button secondary" onClick={this.onClick}>...</button>
        </div>;
    }

    protected renderError(): ReactNode {
        return <div className="error">{this.customError || this.error}</div>;
    }

    @bind
    protected async onClick(): Promise<void> {
        const props = { openLabel: "OK", canSelectMany: false, canSelectFolders: true, canSelectFiles: true };
        const workspace = await this.workspace.workspace;
        const uri = workspace && await this.fileDialog.showOpenDialog(props as OpenFileDialogProps, workspace);
        const uriWorkspace = workspace && new URI(workspace.uri);
        const relativePath = uri && uriWorkspace && uriWorkspace.relative(uri);

        this.customError = relativePath ? undefined : "Path must be within workspace.";

        if (relativePath) this.setValue(relativePath.toString());
        else this.setValue(this.value);
    }

    public static readonly TYPE: string = "de.monticore.lang.monticar.sol.option.types.Path";

    public static createComponent(props: PathProps, container: Container): ReactNode {
        const fileDialog = container.get<FileDialogService>(FileDialogService);
        const workspace = container.get<WorkspaceService>(WorkspaceService);

        return <PathOption fileDialog={fileDialog} workspace={workspace} {...props}/>;
    }
}
