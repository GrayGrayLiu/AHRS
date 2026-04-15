/******************************************************************************
 * @file    Aided_INS_API.h
 * @brief   C与C++的接口
 *
 * @details
 *
 *
 * @author  Gray
 * @email   grayme12345@gmail.com
 * @date    2026/4/7
 *
 * @copyright
 * Copyright (c) 2026 Gray
 *
 * This software is provided "as is" without any warranties.
 *
 ******************************************************************************/

#ifndef AHRS_AIDED_INS_API_HPP
#define AHRS_AIDED_INS_API_HPP

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct Aided_INS Aided_INS;
typedef Aided_INS* Aided_INS_Handle;

Aided_INS_Handle Aided_INS_Create(int id);

#ifdef __cplusplus
}
#endif

#endif //AHRS_AIDED_INS_API_HPP
