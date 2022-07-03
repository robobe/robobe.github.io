---
title: yaml json files and json-schema
tags:
    - json
    - yaml
    - json
    - schema
    - vscode
---

VSCode has the ability to display autocomplete suggestions for JSON and YAML format out of the box.  
It's use `JSON schema` to do it

## JSON Schema
[JSON Schema](https://json-schema.org/) is a specification that allows you to describe the structure of a JSON document and validate documents against that schema.

## VSCode
- Install [YAML red hat](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-yaml)

### settings

```json
"yaml.schemas": {
  "<shema file location>.json": "<file useage by schema>.yaml",
  "<shema file location>.json": "<file useage by schema>.json"
},
```

# References
- [How to create your own auto-completion for JSON and YAML files on VS Code with the help of JSON Schema](https://dev.to/brpaz/how-to-create-your-own-auto-completion-for-json-and-yaml-files-on-vs-code-with-the-help-of-json-schema-k1i)