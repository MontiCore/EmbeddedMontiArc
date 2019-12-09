/*
 * (c) https://github.com/MontiCore/monticore
 */
import { PureComponent, ReactNode } from "react";
import { lazyInject } from "../../common/ioc";
import { Application } from "../application";

import * as React from "react";

import styled from "styled-components";

export class LogoBase extends PureComponent {
    @lazyInject(Application) protected readonly application: Application;

    public render(): ReactNode {
        const logo = this.application.getLogo();

        return <img src={logo} alt="Logo"/>;
    }
}

export const Logo = styled(LogoBase)``;
