/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once

#include "computer/registers.h"

namespace OS {
    /*
        This is the implementation of the FunctionCalling interface for the Linux FastCall calling convention.
    
        The integer and pointer arguments are passed from left to right into the RDI, RSI, RDX, RCX registers
        The return value for integers and pointers is passed in the RAX register.
    
        For floating-point arguments, they are placed inside the XMM0, XMM1, XMM2, ... registers.
        The return value is in the XMM0 register.

        For details see https://en.wikipedia.org/wiki/X86_calling_conventions
        Under the "System V AMD64 ABI" section.
    */
    struct LinuxFastCall {
        Registers &registers;
        
        //Caller
        inline void set_params_64(ulong p1) {
            set_param1_64(p1);
        }
        inline void set_params_64(ulong p1, ulong p2) {
            set_param1_64(p1);
            set_param2_64(p2);
        }
        inline void set_params_64(ulong p1, ulong p2, ulong p3) {
            set_param1_64(p1);
            set_param2_64(p2);
            set_param3_64(p3);
        }
        inline void set_params_64(ulong p1, ulong p2, ulong p3, ulong p4) {
            set_param1_64(p1);
            set_param2_64(p2);
            set_param3_64(p3);
            set_param4_64(p4);
        }
        inline void set_params_32(uint p1) {
            set_param1_32(p1);
        }
        inline void set_params_32(uint p1, uint p2) {
            set_param1_32(p1);
            set_param2_32(p2);
        }
        inline void set_params_32(uint p1, uint p2, uint p3) {
            set_param1_32(p1);
            set_param2_32(p2);
            set_param3_32(p3);
        }
        inline void set_params_32(uint p1, uint p2, uint p3, uint p4) {
            set_param1_32(p1);
            set_param2_32(p2);
            set_param3_32(p3);
            set_param4_32(p4);
        }
        inline void set_params_double(double p1) {
            set_param1_double(p1);
        }
        inline void set_params_double(double p1, double p2) {
            set_param1_double(p1);
            set_param2_double(p2);
        }
        inline void set_params_double(double p1, double p2, double p3) {
            set_param1_double(p1);
            set_param2_double(p2);
            set_param3_double(p3);
        }
        inline void set_params_double(double p1, double p2, double p3, double p4) {
            set_param1_double(p1);
            set_param2_double(p2);
            set_param3_double(p3);
            set_param4_double(p4);
        }
        inline void set_params_float(float p1) {
            set_param1_float(p1);
        }
        inline void set_params_float(float p1, float p2) {
            set_param1_float(p1);
            set_param2_float(p2);
        }
        inline void set_params_float(float p1, float p2, float p3) {
            set_param1_float(p1);
            set_param2_float(p2);
            set_param3_float(p3);
        }
        inline void set_params_float(float p1, float p2, float p3, float p4) {
            set_param1_float(p1);
            set_param2_float(p2);
            set_param3_float(p3);
            set_param4_float(p4);
        }
        inline void set_param1_64(ulong p) {
            registers.set_rdi(p);
        }
        inline void set_param2_64(ulong p) {
            registers.set_rsi(p);
        }
        inline void set_param3_64(ulong p) {
            registers.set_rdx(p);
        }
        inline void set_param4_64(ulong p) {
            registers.set_rcx(p);
        }
        inline void set_param5_64(ulong p)
        {
            registers.set_r8(p);
        }
        inline void set_param6_64(ulong p)
        {
            registers.set_r9(p);
        }
        inline void set_param1_32(uint p) {
            registers.set_rdi(p);
        }
        inline void set_param2_32(uint p) {
            registers.set_rsi(p);
        }
        inline void set_param3_32(uint p) {
            registers.set_rdx(p);
        }
        inline void set_param4_32(uint p) {
            registers.set_rcx(p);
        }
        inline void set_param5_32(uint p)
        {
            registers.set_r8(p);
        }
        inline void set_param6_32(uint p)
        {
            registers.set_r9(p);
        }
        inline void set_param1_double(double p) {
            registers.set_xmm0(p);
        }
        inline void set_param2_double(double p) {
            registers.set_xmm1(p);
        }
        inline void set_param3_double(double p) {
            registers.set_xmm2(p);
        }
        inline void set_param4_double(double p) {
            registers.set_xmm3(p);
        }
        inline void set_param1_float(float p) {
            registers.set_xmm0_f(p);
        }
        inline void set_param2_float(float p) {
            registers.set_xmm1_f(p);
        }
        inline void set_param3_float(float p) {
            registers.set_xmm2_f(p);
        }
        inline void set_param4_float(float p) {
            registers.set_xmm3_f(p);
        }
        inline ulong get_return_64() {
            return registers.get_rax();
        }
        inline uint get_return_32() {
            return (uint)registers.get_rax();
        }
        inline double get_return_double() {
            return registers.get_xmm0();
        }

        inline float get_return_float() {
            return registers.get_xmm0_f();
        }

        inline char get_return_char() {
            return (char)registers.get_rax();
        }




        //Callee
        inline ulong get_param1_64() {
            return registers.get_rdi();
        }
        inline ulong get_param2_64() {
            return registers.get_rsi();
        }
        inline ulong get_param3_64() {
            return registers.get_rdx();
        }
        inline ulong get_param4_64() {
            return registers.get_rcx();
        }
        inline uint get_param1_32() {
            return (uint)registers.get_rdi();
        }
        inline uint get_param2_32() {
            return (uint)registers.get_rsi();
        }
        inline uint get_param3_32() {
            return (uint)registers.get_rdx();
        }
        inline uint get_param4_32() {
            return (uint)registers.get_rcx();
        }
        inline double get_param1_double() {
            return registers.get_xmm0();
        }
        inline double get_param2_double() {
            return registers.get_xmm1();
        }
        inline double get_param3_double() {
            return registers.get_xmm2();
        }
        inline double get_param4_double() {
            return registers.get_xmm3();
        }
        inline float get_param1_float() {
            return registers.get_xmm0_f();
        }
        inline float get_param2_float() {
            return registers.get_xmm1_f();
        }
        inline float get_param3_float() {
            return registers.get_xmm2_f();
        }
        inline float get_param4_float() {
            return registers.get_xmm3_f();
        }
        inline void set_return_64(ulong r) {
            registers.set_rax(r);
        }
        inline void set_return_32(uint r) {
            registers.set_rax(r);
        }
        inline void set_return_double(double r) {
            registers.set_xmm0(r);
        }
        
        LinuxFastCall( Registers &registers ) : registers( registers ) {}
    };
    
}