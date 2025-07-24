/*
 * (c) https://github.com/MontiCore/monticore
 */
import { SnackbarContent } from "@material-ui/core";
import { AbstractMessageBase } from "@embeddedmontiarc/sol-external-core/lib/renderer";
import { Message } from "@theia/core/lib/common/message-service-protocol";
import { bind } from "helpful-decorators";
import { ReactNode } from "react";

import * as React from "react";

import styled from "styled-components";

const Content = styled(SnackbarContent)<{ variable: string }>`
    background-color:var(--theia-${props => props.variable}) !important;
    border-radius:0 !important;
` as any; // tslint:disable-line:no-any

export class PreparationMessageBase extends AbstractMessageBase {
    public constructor(props: {}) {
        super(props);

        this.state = { message: this.notifications.getCurrentMessage() };
    }

    public componentWillMount(): void {
        this.notifications.on("show-message", this.onShowMessage);
    }

    public componentWillUnmount(): void {
        this.notifications.off("show-message", this.onShowMessage);
    }

    public render(): ReactNode {
        return <Content variable={this.getBackgroundVariable()} message={this.renderContentMessage()}/>;
    }

    @bind
    protected onShowMessage(message: Message): void {
        this.setState({ message });
    }
}

export const PreparationMessage = styled(PreparationMessageBase)``;
