[![Build Status](https://travis-ci.org/EmbeddedMontiArc/Struct.svg?branch=master)](https://travis-ci.org/EmbeddedMontiArc/Struct)
[![Build Status](https://circleci.com/gh/EmbeddedMontiArc/Struct/tree/master.svg?style=shield&circle-token=:circle-token)](https://circleci.com/gh/EmbeddedMontiArc/Struct/tree/master)
[![Coverage Status](https://coveralls.io/repos/github/EmbeddedMontiArc/Struct/badge.svg?branch=master)](https://coveralls.io/github/EmbeddedMontiArc/Struct?branch=master)

# Struct
Language for defining C-like structs:
```
struct MyStruct1 {
  Q someField1;
  B someField2;
  MyOtherStruct1 someField3;
}
```

Some constraints:
* one model file -- one struct
* no generics (just as in the good ol' C :smiley:)
* struct references may not form a cycle
