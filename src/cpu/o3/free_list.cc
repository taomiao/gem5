/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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
 * Authors: Kevin Lim
 */

#include "cpu/o3/free_list.hh"

#include "arch/registers.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "debug/FreeList.hh"

UnifiedFreeList::UnifiedFreeList(const std::string &_my_name,
                                 PhysRegFile *_regFile)
    : _name(_my_name), regFile(_regFile)
{
    DPRINTF(FreeList, "Creating new free list object.\n");

    // Have the register file initialize the free list since it knows
    // about its internal organization
    regFile->initFreeList(this);
}
UnifiedFreeList::UnifiedFreeList(const std::string &_my_name, int vir_int_reg,
                                             int vir_float_reg,
                                                                                     int vir_vec_reg,
                                                                                         int vir_vecpred_reg,
                                                                                         int vir_cc_reg,
                                                                                         Enums::VecRegRenameMode vec_mode)
: _name(_my_name), regFile(NULL)
{
    DPRINTF(FreeList, "Creating virtual free list object.\n");

    // Have the register file initialize the free list since it knows
    // about its internal organization
    //regFile->initFreeList(this);
		long flat_reg_num = 0;
        for (long i=0;i<vir_int_reg;i++){
                this->addIntReg((PhysRegId*)flat_reg_num);
				flat_reg_num++;
        }
        for (long i=0;i<vir_float_reg;i++){
                this->addFloatReg((PhysRegIdPtr)flat_reg_num);
				flat_reg_num++;
        }
        for (long i=0;i<vir_vec_reg;i++){
            if (vec_mode == Enums::Full){
                        addVecReg((PhysRegIdPtr)flat_reg_num);
				}
                else{
                        addVecElem((PhysRegIdPtr)flat_reg_num);
					}
			flat_reg_num++;
        }
        for (long i=0;i<vir_vecpred_reg;i++){
                addVecPredReg((PhysRegIdPtr)flat_reg_num);
				flat_reg_num++;
        }
        for (long i=0;i<vir_cc_reg;i++){
                addCCReg((PhysRegIdPtr)flat_reg_num);
				flat_reg_num++;
        }
}
