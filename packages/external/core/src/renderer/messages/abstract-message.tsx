/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Component, ReactNode } from "react";
import { lazyInject } from "../../common";
import { NotificationsClientImpl } from "./notifications-client-impl";
import { Message, MessageClient, MessageType } from "@theia/core/lib/common/message-service-protocol";

import styled, { StyledComponentBase } from "styled-components";
import ErrorIcon from "@material-ui/icons/Error";
import InfoIcon from "@material-ui/icons/Info";
import WarningIcon from "@material-ui/icons/Warning";

import * as React from "react";

const Message = styled.span`display:flex; align-items:center; color:#000;`;

const icon = `font-size:20px;`;
const iconVariant = `opacity:0.9; margin-right:10px;`;

const MessageInfo = styled(InfoIcon)`${icon} ${iconVariant}`;
const MessageError = styled(ErrorIcon)`${icon} ${iconVariant}`;
const MessageWarning = styled(WarningIcon)`${icon} ${iconVariant}`;

/**
 * An interface which represents the state of the [[AbstractMessageBase]].
 */
export interface MessageState {
    /**
     * The [[Message]] to be displayed.
     */
    message?: Message | undefined;
}

export abstract class AbstractMessageBase<P = {}, T extends MessageState = { message: Message | undefined }> extends Component<P, T> {
    @lazyInject(MessageClient)
    protected readonly notifications: NotificationsClientImpl;

    protected get message(): Message | undefined {
        return this.state.message;
    }

    protected get text(): string {
        return this.message ? this.message.text : '';
    }

    protected get type(): MessageType {
        return this.message && this.message.type ? this.message.type : MessageType.Info;
    }

    // tslint:disable-next-line:no-any
    protected getIconComponent(): StyledComponentBase<any, any> {
        switch (this.type) {
            case MessageType.Error: return MessageError;
            case MessageType.Warning: return MessageWarning;
            default: return MessageInfo;
        }
    }

    protected getBackgroundVariable(): string {
        switch (this.type) {
            case MessageType.Error: return "error-color0";
            case MessageType.Warning: return "warn-color0";
            default: return "brand-color0";
        }
    }

    protected renderContentMessage(): ReactNode {
        const MessageIcon = this.getIconComponent();

        return <Message>
            <MessageIcon/>
            {this.text}
        </Message>;
    }
}
