/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef MY_STRING_H_INCLUDED
#define MY_STRING_H_INCLUDED

#include <stdint.h>

#ifndef NULL
#define NULL 0L
#endif

#define TOSTR_(...) #__VA_ARGS__
#define STRINGIFY(x) TOSTR_(x)

#ifdef __cplusplus
extern "C"
{
#endif

int my_strcmp(const char *str1, const char *str2);
void my_strcat(char *str1, const char *str2);
int my_strlen(const char *str);
const char *my_strchr(const char *str, const char c);
int my_ltoa(char *buf, int val, int base);
int my_atoi(const char *str);
char *my_trim(char *str);

/* memcopy: length in own units */
void memcpy8(uint8_t* target, uint8_t *source, int length);
/* memset: length in own units */
void memset8(uint8_t* target, uint8_t value, int length);
/* memcopy: length in own units */
void memcpyu32(uint32_t* target, uint32_t *source, int length);
/* memset: length in own units */
void memsetu32(uint32_t* target, uint32_t value, int length);
/* memcopy: length in own units */
void memcpy32(int* target, int *source, int length);
/* memset: length in own units */
void memset32(int* target, int value, int length);

void my_strcpy(char *str1, const char *str2);

#ifdef __cplusplus
}
#endif

#endif