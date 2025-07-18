/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { CNNTRAINLANG_LANGUAGE_ID, CNNTRAINLANG_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./cnntrainlang-monaco-language";

monaco.languages.register({
    id: CNNTRAINLANG_LANGUAGE_ID,
    extensions: [".cnnt"],
    aliases: [CNNTRAINLANG_LANGUAGE_NAME],
    mimetypes: ["text/x-cnntrainlang"]
});

monaco.languages.onLanguage(CNNTRAINLANG_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(CNNTRAINLANG_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(CNNTRAINLANG_LANGUAGE_ID, monarchLanguage);
});
