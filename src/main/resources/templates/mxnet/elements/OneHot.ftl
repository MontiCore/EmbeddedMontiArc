        ${element.name} = mx.symbol.one_hot(data=${element.inputs[0]},
            indices=mx.symbol.argmax(data=${element.inputs[0]}, axis=1), depth=${element.size}))
            <#include "OutputShape.ftl">