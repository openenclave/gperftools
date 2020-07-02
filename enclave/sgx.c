// Copyright (c) Open Enclave SDK contributors.
// Licensed under the MIT License.

/*
 * Copyright (C) 2011-2020 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

// Based on
// https://github.com/intel/linux-sgx/blob/master/sdk/gperftools/gperftools-2.7/src/base/sgx_utils.cc

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#define OE_UNUSED(x) (void)x

const int ENCLAVE_CPU_COUNT = 4;

const char* TCMallocGetenvSafe(const char* name)
{
    OE_UNUSED(name);
    return NULL;
}

int GetSystemCPUsCount()
{
    return ENCLAVE_CPU_COUNT;
}

size_t write(int fd, const void* buf, size_t count)
{
    OE_UNUSED(fd);
    OE_UNUSED(buf);
    OE_UNUSED(count);
    return 0;
}

int RunningOnValgrind(void)
{
    return NULL;
}

char* getenv(const char* name)
{
    OE_UNUSED(name);
    return NULL;
}


/* Set the spinlock value to 1 and return the old value */
static unsigned int _spin_set_locked(uint32_t* spinlock)
{
    uint32_t value = 1;

    asm volatile("lock xchg %0, %1;"
                 : "=r"(value)     /* %0 */
                 : "m"(*spinlock), /* %1 */
                   "0"(value)      /* also %2 */
                 : "memory");

    return value;
}

static void _acquire_lock(uint32_t* spinlock)
{
    while (_spin_set_locked(spinlock) != 0)
    {
        /* Spin while waiting for spinlock to be released (become 1) */
        while (*spinlock)
        {
            /* Yield to CPU */
            asm volatile("pause");
        }
    }
}

static void _release_unlock(uint32_t* spinlock)
{
    asm volatile("movl %0, %1;"
                 :
                 : "r"(0), "m"(*spinlock) /* %1 */
                 : "memory");
}


static uint8_t* _heap_start;
static uint8_t* _heap_end;
static uint8_t* _heap_next;
static uint32_t  _lock = 0;

void* sbrk(ptrdiff_t increment)
{
    void* ptr = (void*)-1;

    _acquire_lock(&_lock);
    {
	ptrdiff_t remaining;

	if (!_heap_next)
	    _heap_next = _heap_start;

	remaining = _heap_end - _heap_next;

	if (increment <= remaining)
	{
	    ptr = _heap_next;
	    _heap_next += increment;
	}
    }
    _release_lock(&_lock);

    return ptr;
}

void tcmalloc_set_enclave_heap_range(uint8_t* heap_start,
				     uint8_t* heap_end)
{
    _heap_start = heap_start;
    _heap_end = heap_end;
}
