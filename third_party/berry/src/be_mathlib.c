/********************************************************************
** Copyright (c) 2018-2020 Guan Wenliang
** This file is part of the Berry default interpreter.
** skiars@qq.com, https://github.com/Skiars/berry
** See Copyright Notice in the LICENSE file or at
** https://github.com/Skiars/berry/blob/master/LICENSE
********************************************************************/
/* Fantasi trim: the full math module (sin/cos/tan/sqrt/pow/log/...) pulls in
** ~28 KB of libm, which pushed the Proxmark3 firmware past the AT91SAM7S512
** flash-plane boundary (0x140000) into the filesystem region. Only rand/srand
** are kept here - they use plain C stdlib (no libm). Apps that need real math
** can bundle their own native module. If this file is re-synced from upstream
** Berry, re-apply this trim (and re-run `make berry`). */
#include "be_object.h"
#include <stdlib.h>

#if BE_USE_MATH_MODULE

static int m_srand(bvm *vm)
{
    if (be_top(vm) >= 1 && be_isint(vm, 1)) {
        srand((unsigned int)be_toint(vm, 1));
    }
    be_return_nil(vm);
}

static int m_rand(bvm *vm)
{
    be_pushint(vm, rand());
    be_return(vm);
}

#if !BE_USE_PRECOMPILED_OBJECT
be_native_module_attr_table(math) {
    be_native_module_function("srand", m_srand),
    be_native_module_function("rand", m_rand),
};

be_define_native_module(math, NULL);
#else
/* @const_object_info_begin
module math (scope: global, depend: BE_USE_MATH_MODULE) {
    srand, func(m_srand)
    rand, func(m_rand)
}
@const_object_info_end */
#include "../generate/be_fixed_math.h"
#endif

#endif /* BE_USE_MATH_MODULE */
