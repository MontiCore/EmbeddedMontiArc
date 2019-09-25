/*
 * (c) https://github.com/MontiCore/monticore
 */
import { PureComponent, ReactNode } from "react";

import * as React from "react";

import styled from "styled-components";

export const logo = require("../../../src/renderer/images/logo.png");

export class LogoBase extends PureComponent {
    public render(): ReactNode {
        return <img src={logo} alt="Logo"/>;
    }
}

export const Logo = styled(LogoBase)``;
