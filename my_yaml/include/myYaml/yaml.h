#ifndef YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66
#define YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66

#if defined(_MSC_VER) ||                                            \
    (defined(__GNUC__) && (__GNUC__ == 3 && __GNUC_MINOR__ >= 4) || \
     (__GNUC__ >= 4))  // GCC supports "pragma once" correctly since 3.4
#pragma once
#endif

#include "my_yaml/include/myYaml/parser.h"
#include "my_yaml/include/myYaml/emitter.h"
#include "my_yaml/include/myYaml/emitterstyle.h"
#include "my_yaml/include/myYaml/stlemitter.h"
#include "my_yaml/include/myYaml/exceptions.h"

#include "my_yaml/include/myYaml/node/node.h"
#include "my_yaml/include/myYaml/node/impl.h"
#include "my_yaml/include/myYaml/node/convert.h"
#include "my_yaml/include/myYaml/node/iterator.h"
#include "my_yaml/include/myYaml/node/detail/impl.h"
#include "my_yaml/include/myYaml/node/parse.h"
#include "my_yaml/include/myYaml/node/emit.h"
#include "my_yaml/include/myYaml/yaml_eigen.h"

#endif  // YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66
