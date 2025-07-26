/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { EMBEDDEDMONTIARCMATH_LANGUAGE_ID, EMBEDDEDMONTIARCMATH_LANGUAGE_NAME } from "../common";

@injectable()
export class EmbeddedMontiArcMathLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = EMBEDDEDMONTIARCMATH_LANGUAGE_ID;
    public readonly name: string = EMBEDDEDMONTIARCMATH_LANGUAGE_NAME;

    protected onRuntimeInitialized(): void {
        const original = this.parse;

        this.parse = this.extendParse(original);

        super.onRuntimeInitialized();
    }

    protected extendParse(original: (docValue: string) => void): (docValue: string) => void {
        return (docValue: string) => { original(this.removeNoise(docValue)); };
    }

    protected removeNoise(docValue: string): string {
        return this.removeImplementation(this.removeComments(docValue));
    }

    protected removeComments(docValue: string): string {
        return docValue.replace(/((['"])(?:(?!\2|\\).|\\.)*\2)|\/\/[^\n]*|\/\*(?:[^*]|\*(?!\/))*\*\//g, this.replaceCommentMatch);
    }

    protected removeImplementation(docValue: string, initial?: number): string {
        const start = docValue.indexOf("implementation", initial || 0);
        const middle = start === -1 ? -1 : docValue.indexOf("Math", start);

        if (middle === -1) return docValue;
        else return this.removeImplementation(this.doRemoveImplementation(docValue, start), start);
    }

    protected replaceCommentMatch(match: string): string {
        const length = match.length;

        let modMatch = "";

        for (let i = 0; i < length; i++) {
            modMatch += match[i] === ' ' || match[i] === '\n' ? match[i] : ' ';
        }

        return modMatch;
    }

    protected doRemoveImplementation(docValue: string, index: number): string {
        const length = docValue.length;

        let bracketCounter = 0;
        let inBody = false;

        for (let i = index; i < length; i++) {
            if (docValue[i] === '{') {
                bracketCounter++;
                inBody = true;
            } else if (docValue[i] === '}') {
                bracketCounter--;
            }

            docValue = this.replaceAt(docValue, i, docValue[i] !== '\n' ? ' ' : '\n');

            if (inBody && bracketCounter === 0) return docValue;
        }

        return docValue;
    }

    protected replaceAt(string: string, index: number, replacement: string): string {
        return string.substr(0, index) + replacement + string.substr(index + replacement.length);
    }
}
