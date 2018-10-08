/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { TAGGING_LANGUAGE_ID, TAGGING_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./tagging-monaco-language";

monaco.languages.register({
    id: TAGGING_LANGUAGE_ID,
    extensions: [".tag"],
    aliases: [TAGGING_LANGUAGE_NAME],
    mimetypes: ["text/x-tagging"]
});

monaco.languages.onLanguage(TAGGING_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(TAGGING_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(TAGGING_LANGUAGE_ID, monarchLanguage);
});
