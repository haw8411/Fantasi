#include "be_constobj.h"

static be_define_const_map_slots(m_libmath_map) {
    { be_const_key(rand, -1), be_const_func(m_rand) },
    { be_const_key(srand, -1), be_const_func(m_srand) },
};

static be_define_const_map(
    m_libmath_map,
    2
);

static be_define_const_module(
    m_libmath,
    "math"
);

BE_EXPORT_VARIABLE be_define_const_native_module(math);
