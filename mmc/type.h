#ifndef _TYPE_H_
#define _TYPE_H_

typedef unsigned char u8;
typedef unsigned int u32;
typedef int s32;
typedef unsigned long ulong;
typedef unsigned long long u64;
typedef unsigned int uint;
typedef __signed__ int __s32;
typedef unsigned int __u32;
typedef unsigned int size_t;
typedef ulong lbaint_t;

#define  readl(a)   *(volatile uint *)(ulong)(a)
#define  writel(v, c) *(volatile uint *)(ulong)(c) = (v)

#endif
