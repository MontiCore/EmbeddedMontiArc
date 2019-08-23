import { FileDialogService, OpenFileDialogProps } from "@theia/filesystem/lib/browser";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { boundMethod } from "autobind-decorator";
import { interfaces } from "inversify";
import * as React from "react";
import URI from "@theia/core/lib/common/uri";
import { ValueComponent, ValueComponentProps } from "./value-component";
import ReactNode = React.ReactNode;
import RefObject = React.RefObject;
import Container = interfaces.Container;

/**
 * Represents the props passed to a path component.
 */
export interface PathComponentProps extends ValueComponentProps {
    readonly label: string;
    readonly workspace: WorkspaceService;
    readonly fileDialog: FileDialogService;
}

/**
 * Represents the state of a path component.
 */
export interface PathComponentState {
    uri: URI | undefined;
}

/**
 * Implementation of a path component.
 */
export class PathComponent extends ValueComponent<PathComponentProps, PathComponentState> {
    public constructor(props: PathComponentProps) {
        super(props);

        this.state = { uri: undefined };
    }

    protected get workspace(): WorkspaceService {
        return this.props.workspace;
    }

    protected get fileDialog(): FileDialogService {
        return this.props.fileDialog;
    }

    protected get label(): string {
        return `${this.props.label}${this.required ? '*' : ''}`;
    }

    protected get uri(): URI | undefined {
        return this.state.uri;
    }

    public get valid(): boolean {
        return !this.required || (this.required && this.filled);
    }

    public get filled(): boolean {
        return this.uri !== undefined;
    }

    public get value(): string {
        return this.uri ? this.uri.toString(true) : '';
    }

    protected get className(): string {
        const invalid = this.valid ? '' : "invalid";

        return `sol-runtime-components component ${invalid} path-component`;
    }

    public render(): ReactNode {
        return <div className={this.className}>
            <span className="label">{this.label}</span>
            <input type="text" value={this.value} readOnly={true}/>
            <button className="theia-button secondary" onClick={this.onClick}>...</button>
        </div>;
    }

    @boundMethod
    protected async onClick(): Promise<void> {
        const workspace = await this.workspace.workspace;
        const props = { openLabel: "OK", canSelectMany: false } as OpenFileDialogProps;

        if (workspace) {
            const uri = await this.fileDialog.showOpenDialog(props, workspace);

            if (uri) this.setState({ uri });
        }
    }

    public static readonly TYPE: string = "path";

    public static createComponent(ref: RefObject<PathComponent>, props: PathComponentProps, container: Container): ReactNode {
        const fileDialog = container.get<FileDialogService>(FileDialogService);
        const workspace = container.get<WorkspaceService>(WorkspaceService);

        return <PathComponent ref={ref} fileDialog={fileDialog} workspace={workspace} {...props}/>;
    }
}
