/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { OCL_LANGUAGE_ID, OCL_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./ocl-monaco-language";

monaco.languages.register({
    id: OCL_LANGUAGE_ID,
    extensions: [".ocl"],
    aliases: [OCL_LANGUAGE_NAME],
    mimetypes: ["text/x-ocl"]
});

monaco.languages.onLanguage(OCL_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(OCL_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(OCL_LANGUAGE_ID, monarchLanguage);
});
