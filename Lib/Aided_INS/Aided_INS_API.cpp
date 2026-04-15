//
// Created by Gray on 2026/4/7.
//

#include "Aided_INS_API.h"
#include "Aided_INS.hpp"

Aided_INS_Handle Aided_INS_Create(const int id)
{
    const Aided_INS_Handle aided_INS_Handle = new Aided_INS(id);

    return aided_INS_Handle;
}
