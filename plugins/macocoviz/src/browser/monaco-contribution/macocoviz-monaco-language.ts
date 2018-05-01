/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/// <reference types="monaco-editor-core/monaco" />

export const configuration: monaco.languages.LanguageConfiguration = {
    // the default separators except `@$`
    wordPattern: /(-?\d*\.\d\w*)|([^\`\~\!\#\%\^\&\*\(\)\-\=\+\[\{\]\}\\\|\;\:\'\"\,\.\<\>\/\?\s]+)/g,
    comments: {
        lineComment: "//",
        blockComment: ["/*", "*/"],
    },
    brackets: [['{', '}'], ['[', ']'], ['(', ')'], ['<', '>']],
    autoClosingPairs: [
        { open: '"', close: '"', notIn: ["string", "comment"] },
        { open: '\'', close: '\'', notIn: ["string", "comment"] },
        { open: '{', close: '}', notIn: ["string", "comment"] },
        { open: '[', close: ']', notIn: ["string", "comment"] },
        { open: '(', close: ')', notIn: ["string", "comment"] },
        { open: '<', close: '>', notIn: ["string", "comment"] }
    ]
};

export const monarchLanguage = <monaco.languages.IMonarchLanguage>{
    defaultToken: '',
    tokenPostfix: ".viz",

    keywords: [
        "vizualization", "componentType", "flexContainerConfig", "children", "styles", "background", "color", "font",
        "size", "styleTemplate", "child", "inputs", "outputs", "pipes", "fxLayoutType", "fxFlex", "name",
        "placeHolder", "label", "class", "attributes", "fxLayoutAlign", "flexElementConfig", "fxLayoutWrap"
    ],

    operators: [
        "start", "center", "end", "white", "black", "red", "green", "TextInputComponent", "MoneyInputComponent",
        "PercentageInputComponent", "AutoCompleteComponent", "CardComponent", "TableComponent", "PieChartComponent",
        "DOMContainerComponent", "DOMElementComponent", "auto"
    ],

    // we include these common regular expressions
    symbols: /[=><!~?:&|+\-*\/\^%]+/,
    escapes: /\\(?:[abfnrtv\\"']|x[0-9A-Fa-f]{1,4}|u[0-9A-Fa-f]{4}|U[0-9A-Fa-f]{8})/,
    digits: /\d+(_+\d+)*/,
    octaldigits: /[0-7]+(_+[0-7]+)*/,
    binarydigits: /[0-1]+(_+[0-1]+)*/,
    hexdigits: /[[0-9a-fA-F]+(_+[0-9a-fA-F]+)*/,

    // The main tokenizer for our languages
    tokenizer: {
        root: [
            // identifiers and keywords
            [/[a-zA-Z_$][\w$]*/, {
                cases: {
                    "@keywords": { token: "keyword.$0" },
                    "@default": "identifier"
                }
            }],

            // whitespace
            { include: "@whitespace" },

            // delimiters and operators
            [/[{}()\[\]]/, "@brackets"],
            [/[<>](?!@symbols)/, "@brackets"],
            [/@symbols/, {
                cases: {
                    "@operators": "delimiter",
                    "@default": ''
                }
            }],

            // @ annotations.
            [/@\s*[a-zA-Z_\$][\w\$]*/, "annotation"],

            // numbers
            [/(@digits)[eE]([\-+]?(@digits))?[fFdD]?/, "number.float"],
            [/(@digits)\.(@digits)([eE][\-+]?(@digits))?[fFdD]?/, "number.float"],
            [/0[xX](@hexdigits)[Ll]?/, "number.hex"],
            [/0(@octaldigits)[Ll]?/, "number.octal"],
            [/0[bB](@binarydigits)[Ll]?/, "number.binary"],
            [/(@digits)[fFdD]/, "number.float"],
            [/(@digits)[lL]?/, "number"],

            // delimiter: after number because of .\d floats
            [/[;,.]/, "delimiter"],

            // strings
            [/"([^"\\]|\\.)*$/, "string.invalid"],  // non-teminated string
            [/"/, "string", "@string"],

            // characters
            [/'[^\\']'/, "string"],
            [/(')(@escapes)(')/, ["string", "string.escape", "string"]],
            [/'/, "string.invalid"]
        ],

        whitespace: [
            [/[ \t\r\n]+/, ''],
            [/\/\*\*(?!\/)/, "comment.doc", "@javadoc"],
            [/\/\*/, "comment", "@comment"],
            [/\/\/.*$/, "comment"],
        ],

        comment: [
            [/[^\/*]+/, "comment"],
            [/\*\//, "comment", "@pop"],
            [/[\/*]/, "comment"]
        ],

        javadoc: [
            [/[^\/*]+/, 'comment.doc'],
            [/\/\*/, 'comment.doc.invalid'],
            [/\*\//, 'comment.doc', '@pop'],
            [/[\/*]/, 'comment.doc']
        ],

        string: [
            [/[^\\"]+/, "string"],
            [/@escapes/, "string.escape"],
            [/\\./, "string.escape.invalid"],
            [/"/, "string", "@pop"]
        ],
    },
};
