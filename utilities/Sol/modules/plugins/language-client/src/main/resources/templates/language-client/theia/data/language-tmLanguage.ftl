<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="repository" type="java.lang.String" -->
<#-- @ftlvariable name="patterns" type="java.lang.String" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration" -->
${tc.signature("patterns", "repository")}
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign rootSymbol = glex.getGlobalVar("rootSymbol")>
<#assign grammarName = configuration.getGrammarName()>
<#assign extension = rootSymbol.getExtension().orElse(".")>
{
  "information_for_contributors": [],
  "name": "${grammarName}",
  "scopeName": "source.${extension}",
  "patterns": [{
      "include": "#comments"
  }, {
      "include": "#numbers"
  }, {
      "include": "#strings"
  }, {
      "include": "#constants"
  }, ${patterns}],
  "repository": {
    "comments-inline": {
      "patterns": [
        {
          "begin": "/\\*",
          "captures": {
            "0": {
              "name": "punctuation.definition.comment${extension}"
            }
          },
          "end": "\\*/",
          "name": "comment.block${extension}"
        },
        {
          "begin": "(^[ \\t]+)?(?=//)",
          "beginCaptures": {
            "1": {
              "name": "punctuation.whitespace.comment.leading${extension}"
            }
          },
          "end": "(?!\\G)",
          "patterns": [
            {
              "begin": "//",
              "beginCaptures": {
                "0": {
                  "name": "punctuation.definition.comment${extension}"
                }
              },
              "end": "\\n",
              "name": "comment.line.double-slash${extension}"
            }
          ]
        }
      ]
    },
    "comments": {
      "patterns": [
        {
          "captures": {
            "0": {
              "name": "punctuation.definition.comment${extension}"
            }
          },
          "match": "/\\*\\*/",
          "name": "comment.block.empty${extension}"
        },
        {
          "include": "#comments-inline"
        }
      ]
    },
    "strings": {
      "patterns": [
        {
          "begin": "\"",
          "beginCaptures": {
            "0": {
              "name": "punctuation.definition.string.begin${extension}"
            }
          },
          "end": "\"",
          "endCaptures": {
            "0": {
              "name": "punctuation.definition.string.end${extension}"
            }
          },
          "name": "string.quoted.double${extension}",
          "patterns": [
            {
              "match": "\\\\.",
              "name": "constant.character.escape${extension}"
            }
          ]
        },
        {
          "begin": "'",
          "beginCaptures": {
            "0": {
              "name": "punctuation.definition.string.begin${extension}"
            }
          },
          "end": "'",
          "endCaptures": {
            "0": {
              "name": "punctuation.definition.string.end${extension}"
            }
          },
          "name": "string.quoted.single${extension}",
          "patterns": [
            {
              "match": "\\\\.",
              "name": "constant.character.escape${extension}"
            }
          ]
        }
      ]
    },
    "numbers": {
      "patterns": [
        {
          "match": "[0-9]+",
          "name": "constant.numeric.decimal${extension}"
        }
      ]
    },
    "constants": {
      "match": "\\b(true|false)\\b",
      "name": "constant.language${extension}"
    },
    ${repository}
  }
}
