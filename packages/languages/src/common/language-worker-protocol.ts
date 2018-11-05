/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

export const languageWorkerPath = "/services/language-worker";

export const LanguageWorker = Symbol("LanguageWorker");

export interface Position {
    sl?: number;
    el?: number;
    sc?: number;
    ec?: number;
}

export interface AnalyzeItem {
    pos?: Position;
    type?: string;
    message?: string;
}

export interface OutlineItem {
    icon?: string;
    name?: string;
    pos?: Position;
    displayPos?: Position;
    items?: OutlineItem[];
    isUnordered?: boolean;
}

export interface LanguageWorker {
    parse(id: string, docValue: string): Promise<void>;
    analyze(id: string): Promise<AnalyzeItem[]>;
    outline(id: string): Promise<OutlineItem[]>;
}
