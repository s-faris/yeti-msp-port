/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ti/driverlib/dl_crcp.h>

#ifdef __MSPM0_HAS_CRCP__

uint32_t DL_CRCP_calculateBlock32(
    CRCP_Regs* crcp, uint32_t seed, const uint32_t* ptr, uint32_t size)
{
    uint32_t i;

    DL_CRCP_setSeed32(crcp, seed);

    for (i = 0; i < size; i++) {
        DL_CRCP_feedData32(crcp, ptr[i]);
    }

    return (DL_CRCP_getResult32(crcp));
}

uint32_t DL_CRCP_calculateMemoryRange32(
    CRCP_Regs* crcp, uint32_t seed, uint32_t* ptrStart, const uint32_t* ptrEnd)
{
    DL_CRCP_setSeed32(crcp, seed);

    uint32_t* ptr = ptrStart;

    while (ptr <= ptrEnd) {
        DL_CRCP_feedData32(crcp, (uint32_t) *ptr);
        ptr = ptr + 1;
    }

    return (DL_CRCP_getResult32(crcp));
}

uint16_t DL_CRCP_calculateBlock16(
    CRCP_Regs* crcp, uint16_t seed, const uint16_t* ptr, uint16_t size)
{
    uint32_t i;

    DL_CRCP_setSeed16(crcp, seed);

    for (i = 0; i < size; i++) {
        DL_CRCP_feedData16(crcp, ptr[i]);
    }

    return ((uint16_t) DL_CRCP_getResult16(crcp));
}

uint16_t DL_CRCP_calculateMemoryRange16(
    CRCP_Regs* crcp, uint16_t seed, uint16_t* ptrStart, const uint16_t* ptrEnd)
{
    DL_CRCP_setSeed16(crcp, seed);

    uint16_t* ptr = ptrStart;

    while (ptr <= ptrEnd) {
        DL_CRCP_feedData16(crcp, (uint16_t) *ptr);
        ptr = ptr + 1;
    }

    return ((uint16_t) DL_CRCP_getResult16(crcp));
}

#endif /* __MSPM0_HAS_CRCP__ */
