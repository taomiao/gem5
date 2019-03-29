/*
 * Copyright (c) 2010, 2016 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#ifndef __CPU_O3_DYN_INST_HH__
#define __CPU_O3_DYN_INST_HH__

#include <array>

#include "arch/isa_traits.hh"
#include "config/the_isa.hh"
#include "cpu/base_dyn_inst.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/isa_specific.hh"
#include "cpu/reg_class.hh"
#include "debug/O3CPU.hh"

class Packet;

template <class Impl>
class BaseO3DynInst : public BaseDynInst<Impl>
{
  public:
    /** Typedef for the CPU. */
    typedef typename Impl::O3CPU O3CPU;

    /** Binary machine instruction type. */
    typedef TheISA::MachInst MachInst;
    /** Register types. */
    using VecRegContainer = TheISA::VecRegContainer;
    using VecElem = TheISA::VecElem;
    static constexpr auto NumVecElemPerVecReg = TheISA::NumVecElemPerVecReg;
    using VecPredRegContainer = TheISA::VecPredRegContainer;

    enum {
        MaxInstSrcRegs = TheISA::MaxInstSrcRegs,        //< Max source regs
        MaxInstDestRegs = TheISA::MaxInstDestRegs       //< Max dest regs
    };

  public:
    /** BaseDynInst constructor given a binary instruction. */
    BaseO3DynInst(const StaticInstPtr &staticInst, const StaticInstPtr
            &macroop, TheISA::PCState pc, TheISA::PCState predPC,
            InstSeqNum seq_num, O3CPU *cpu);

    /** BaseDynInst constructor given a static inst pointer. */
    BaseO3DynInst(const StaticInstPtr &_staticInst,
                  const StaticInstPtr &_macroop);

    ~BaseO3DynInst();

    /** Executes the instruction.*/
    Fault execute();

    /** Initiates the access.  Only valid for memory operations. */
    Fault initiateAcc();

    /** Completes the access.  Only valid for memory operations. */
    Fault completeAcc(PacketPtr pkt);

  private:
    /** Initializes variables. */
    void initVars();

  protected:
    /** Explicitation of dependent names. */
    using BaseDynInst<Impl>::cpu;
    using BaseDynInst<Impl>::_virSrcRegIdx;
    using BaseDynInst<Impl>::_virDestRegIdx;

    using BaseDynInst<Impl>::_physSrcRegIdx;
    using BaseDynInst<Impl>::_physDestRegIdx;


    /** Values to be written to the destination misc. registers. */
    std::array<RegVal, TheISA::MaxMiscDestRegs> _destMiscRegVal;

    /** Indexes of the destination misc. registers. They are needed to defer
     * the write accesses to the misc. registers until the commit stage, when
     * the instruction is out of its speculative state.
     */
    std::array<short, TheISA::MaxMiscDestRegs> _destMiscRegIdx;

    /** Number of destination misc. registers. */
    uint8_t _numDestMiscRegs;


  public:
#if TRACING_ON
    /** Tick records used for the pipeline activity viewer. */
    Tick fetchTick;      // instruction fetch is completed.
    int32_t decodeTick;  // instruction enters decode phase
    int32_t renameTick;  // instruction enters rename phase
    int32_t dispatchTick;
    int32_t issueTick;
    int32_t completeTick;
    int32_t commitTick;
    int32_t storeTick;
#endif

    /** Reads a misc. register, including any side-effects the read
     * might have as defined by the architecture.
     */
    RegVal
    readMiscReg(int misc_reg) override
    {
        return this->cpu->readMiscReg(misc_reg, this->threadNumber);
    }

    /** Sets a misc. register, including any side-effects the write
     * might have as defined by the architecture.
     */
    void
    setMiscReg(int misc_reg, RegVal val) override
    {
        /** Writes to misc. registers are recorded and deferred until the
         * commit stage, when updateMiscRegs() is called. First, check if
         * the misc reg has been written before and update its value to be
         * committed instead of making a new entry. If not, make a new
         * entry and record the write.
         */
        for (int idx = 0; idx < _numDestMiscRegs; idx++) {
            if (_destMiscRegIdx[idx] == misc_reg) {
               _destMiscRegVal[idx] = val;
               return;
            }
        }

        assert(_numDestMiscRegs < TheISA::MaxMiscDestRegs);
        _destMiscRegIdx[_numDestMiscRegs] = misc_reg;
        _destMiscRegVal[_numDestMiscRegs] = val;
        _numDestMiscRegs++;
    }

    /** Reads a misc. register, including any side-effects the read
     * might have as defined by the architecture.
     */
    RegVal
    readMiscRegOperand(const StaticInst *si, int idx) override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isMiscReg());
        return this->cpu->readMiscReg(reg.index(), this->threadNumber);
    }

    /** Sets a misc. register, including any side-effects the write
     * might have as defined by the architecture.
     */
    void
    setMiscRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isMiscReg());
        setMiscReg(reg.index(), val);
    }

    /** Called at the commit stage to update the misc. registers. */
    void
    updateMiscRegs()
    {
        // @todo: Pretty convoluted way to avoid squashing from happening when
        // using the TC during an instruction's execution (specifically for
        // instructions that have side-effects that use the TC).  Fix this.
        // See cpu/o3/dyn_inst_impl.hh.
        bool no_squash_from_TC = this->thread->noSquashFromTC;
        this->thread->noSquashFromTC = true;

        for (int i = 0; i < _numDestMiscRegs; i++)
            this->cpu->setMiscReg(
                _destMiscRegIdx[i], _destMiscRegVal[i], this->threadNumber);

        this->thread->noSquashFromTC = no_squash_from_TC;
    }

    void forwardOldRegs()
    {

        for (int idx = 0; idx < this->numDestRegs(); idx++) {

                        DPRINTF(O3CPU,"set int reg operand id %d\n",idx);
            VirsRegIdPtr prev_vir_reg = this->prevDestRegIdx(idx);
            const RegId& original_dest_reg =
                this->staticInst->destRegIdx(idx);
                        PhysRegIdPtr prev_phys_reg = this->cpu->lookup(this->threadNumber,original_dest_reg,prev_vir_reg);
                        DPRINTF(O3CPU,"set int reg operand id %d prev_vir_reg %d prev_phys_reg %d\n",idx,(long)prev_vir_reg,prev_phys_reg->index());
            switch (original_dest_reg.classValue()) {
              case IntRegClass:
                this->setIntRegOperand(this->staticInst.get(), idx,
                               this->cpu->readIntReg(prev_phys_reg));
                break;
              case FloatRegClass:
                this->setFloatRegOperandBits(this->staticInst.get(), idx,
                               this->cpu->readFloatReg(prev_phys_reg));
                break;
              case VecRegClass:
                this->setVecRegOperand(this->staticInst.get(), idx,
                               this->cpu->readVecReg(prev_phys_reg));
                break;
              case VecElemClass:
                this->setVecElemOperand(this->staticInst.get(), idx,
                               this->cpu->readVecElem(prev_phys_reg));
                break;
              case VecPredRegClass:
                this->setVecPredRegOperand(this->staticInst.get(), idx,
                               this->cpu->readVecPredReg(prev_phys_reg));
                break;
              case CCRegClass:
                this->setCCRegOperand(this->staticInst.get(), idx,
                               this->cpu->readCCReg(prev_phys_reg));
                break;
              case MiscRegClass:
                // no need to forward misc reg values
                break;
              default:
                panic("Unknown register class: %d",
                        (int)original_dest_reg.classValue());
            }
        }
    }
    /** Calls hardware return from error interrupt. */
    Fault hwrei() override;
    /** Traps to handle specified fault. */
    void trap(const Fault &fault);
    bool simPalCheck(int palFunc) override;

    /** Emulates a syscall. */
    void syscall(int64_t callnum, Fault *fault) override;

  public:

    // The register accessor methods provide the index of the
    // instruction's operand (e.g., 0 or 1), not the architectural
    // register index, to simplify the implementation of register
    // renaming.  We find the architectural register index by indexing
    // into the instruction's own operand index table.  Note that a
    // raw pointer to the StaticInst is provided instead of a
    // ref-counted StaticInstPtr to redice overhead.  This is fine as
    // long as these methods don't copy the pointer into any long-term
    // storage (which is pretty hard to imagine they would have reason
    // to do).

        void reWritePhysRegs(VirsRegIdPtr vir_dst_reg,PhysRegIdPtr dest_reg)
        {
                for (int i=0;i<this->numSrcRegs();i++){
                        if (this->_virSrcRegIdx[i]==vir_dst_reg){
                                DPRINTF(O3CPU,"reWritePhysRegs rewrite vir_reg %d to phys_reg %d\n",(long)vir_dst_reg, dest_reg->index());
                                assert(this->_physSrcRegIdx[i]==NULL);
                                this->_physSrcRegIdx[i] = dest_reg;
                        }
                }
        }

    RegVal
    readIntRegOperand(const StaticInst *si, int idx) override
    {
        //	const RegId& arch_reg = si->srcRegIdx(idx);
        //	VirsRegIdPtr vir_reg = (this->cpu)->lookup(this->threadNumber,arch_reg);
        //	PhysRegIdPtr phys_reg = (this->cpu)->lookup(this->threadNumber,arch_reg,vir_reg);
        //	DPRINTF(O3CPU," arch_reg %d vir_reg %d\n",arch_reg.index(),(long)vir_reg);
                DPRINTF(O3CPU,"readIntRegOperand phys_reg is %d \n",this->_physSrcRegIdx[idx]->index());
                assert(this->_physSrcRegIdx[idx] != NULL);
        //	this->_physSrcRegIdx[idx] = phys_reg;
        return this->cpu->readIntReg(this->_physSrcRegIdx[idx]);
    }

    RegVal
    readFloatRegOperandBits(const StaticInst *si, int idx) override
    {
        return this->cpu->readFloatReg(this->_physSrcRegIdx[idx]);
    }

    const VecRegContainer&
    readVecRegOperand(const StaticInst *si, int idx) const override
    {
        return this->cpu->readVecReg(this->_physSrcRegIdx[idx]);
    }

    /**
     * Read destination vector register operand for modification.
     */
    VecRegContainer&
    getWritableVecRegOperand(const StaticInst *si, int idx) override
    {
                if (this->_physDestRegIdx[idx]==NULL){
                        panic("physical  reg is not allocated\n");
                }
        return this->cpu->getWritableVecReg(this->_physDestRegIdx[idx]);
    }

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector 8bit operand. */
    ConstVecLane8
    readVec8BitLaneOperand(const StaticInst *si, int idx) const override
    {
        return cpu->template readVecLane<uint8_t>(_physSrcRegIdx[idx]);
    }

    /** Reads source vector 16bit operand. */
    ConstVecLane16
    readVec16BitLaneOperand(const StaticInst *si, int idx) const override
    {
        return cpu->template readVecLane<uint16_t>(_physSrcRegIdx[idx]);
    }

    /** Reads source vector 32bit operand. */
    ConstVecLane32
    readVec32BitLaneOperand(const StaticInst *si, int idx) const override
    {
        return cpu->template readVecLane<uint32_t>(_physSrcRegIdx[idx]);
    }

    /** Reads source vector 64bit operand. */
    ConstVecLane64
    readVec64BitLaneOperand(const StaticInst *si, int idx) const override
    {
        return cpu->template readVecLane<uint64_t>(_physSrcRegIdx[idx]);
    }

    /** Write a lane of the destination vector operand. */
    template <typename LD>
    void
    setVecLaneOperandT(const StaticInst *si, int idx, const LD& val)
    {
                if (_physDestRegIdx[idx]==NULL){
                        panic("physical reg is not allocated 0\n");
                }
        return cpu->template setVecLane(_physDestRegIdx[idx], val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::Byte>& val) override
    {
        return setVecLaneOperandT(si, idx, val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::TwoByte>& val) override
    {
        return setVecLaneOperandT(si, idx, val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::FourByte>& val) override
    {
        return setVecLaneOperandT(si, idx, val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::EightByte>& val) override
    {
        return setVecLaneOperandT(si, idx, val);
    }
    /** @} */

    VecElem readVecElemOperand(const StaticInst *si, int idx) const override
    {
        return this->cpu->readVecElem(this->_physSrcRegIdx[idx]);
    }

    const VecPredRegContainer&
    readVecPredRegOperand(const StaticInst *si, int idx) const override
    {
        return this->cpu->readVecPredReg(this->_physSrcRegIdx[idx]);
    }

    VecPredRegContainer&
    getWritableVecPredRegOperand(const StaticInst *si, int idx) override
    {
                if (this->_physDestRegIdx[idx]==NULL){
                        panic("physical reg is not allocated 1\n");
                }
        return this->cpu->getWritableVecPredReg(this->_physDestRegIdx[idx]);
    }

    RegVal
    readCCRegOperand(const StaticInst *si, int idx) override
    {
        return this->cpu->readCCReg(this->_physSrcRegIdx[idx]);
    }

    /** @todo: Make results into arrays so they can handle multiple dest
     *  registers.
     */
    void
    setIntRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
                if (this->_physDestRegIdx[idx]!=NULL){
                        const RegId& arch_reg = si->destRegIdx(idx);
                        DPRINTF(O3CPU,"the arch_reg is %d , the vir reg is %d\n",arch_reg.index(),(long)(this->_virDestRegIdx[idx]));
                        PhysRegIdPtr phys_reg = this->cpu->lookup(this->threadNumber,arch_reg,this->_virDestRegIdx[idx]);
                        if (phys_reg!=NULL){
                                DPRINTF(O3CPU,"get dest phys reg is %d\n",phys_reg->index());
                        }
                        else{
                                DPRINTF(O3CPU,"phys_reg is NULL,do not know where the phys reg comes!!!\n");
                        }
                }

                if (true){//this->_physDestRegIdx[idx]==NULL){
                        // get a physical reg
                        const RegId& arch_reg = si->destRegIdx(idx);
                        DPRINTF(O3CPU,"the arch_reg is %d , the vir reg is %d\n",arch_reg.index(),(long)(this->_virDestRegIdx[idx]));
                        PhysRegIdPtr phys_reg = this->cpu->getPhysReg(this->threadNumber,arch_reg,this->_virDestRegIdx[idx]);
                        DPRINTF(O3CPU,"get dest phys reg is %d\n",phys_reg->index());
                        this->_physDestRegIdx[idx] = phys_reg;
                        //panic("physical reg is not allocated 2\n");
                }
                DPRINTF(O3CPU,"regval is %d\n",val);
        this->cpu->setIntReg(this->_physDestRegIdx[idx], val);
                this->dump();
                //std::cout<<"test1"<<std::endl;
        //BaseDynInst<Impl>::setIntRegOperand(si, idx, val);
                //std::cout<<"test2"<<std::endl;
    }

    void
    setFloatRegOperandBits(const StaticInst *si, int idx, RegVal val) override
    {
                if (this->_physDestRegIdx[idx]==NULL){
                        // get a physical reg
                        const RegId& arch_reg = si->destRegIdx(idx);
                        DPRINTF(O3CPU,"the arch_reg is %d , the vir reg is %d\n",arch_reg.index(),(long)(this->_virDestRegIdx[idx]));
                        PhysRegIdPtr phys_reg = this->cpu->getPhysReg(this->threadNumber,arch_reg,this->_virDestRegIdx[idx]);
                        DPRINTF(O3CPU,"get dest phys reg is %d\n",phys_reg->index());
                        this->_physDestRegIdx[idx] = phys_reg;
                        //panic("physical reg is not allocated 2\n");
                }
        this->cpu->setFloatReg(this->_physDestRegIdx[idx], val);
        BaseDynInst<Impl>::setFloatRegOperandBits(si, idx, val);
    }

    void
    setVecRegOperand(const StaticInst *si, int idx,
                     const VecRegContainer& val) override
    {
                if (this->_physDestRegIdx[idx]==NULL){
                        panic("physical reg is not allocated 3\n");
                }
        this->cpu->setVecReg(this->_physDestRegIdx[idx], val);
        BaseDynInst<Impl>::setVecRegOperand(si, idx, val);
    }

    void setVecElemOperand(const StaticInst *si, int idx,
                           const VecElem val) override
    {
        int reg_idx = idx;
        this->cpu->setVecElem(this->_physDestRegIdx[reg_idx], val);
        BaseDynInst<Impl>::setVecElemOperand(si, idx, val);
    }

    void
    setVecPredRegOperand(const StaticInst *si, int idx,
                         const VecPredRegContainer& val) override
    {
                if (this->_physDestRegIdx[idx]==NULL){
                        panic("physical reg is not allocated\n");
                }
        this->cpu->setVecPredReg(this->_physDestRegIdx[idx], val);
        BaseDynInst<Impl>::setVecPredRegOperand(si, idx, val);
    }

    void setCCRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
                if (this->_physDestRegIdx[idx]==NULL){
                        panic("preg is not allocated\n");
                }
        this->cpu->setCCReg(this->_physDestRegIdx[idx], val);
        BaseDynInst<Impl>::setCCRegOperand(si, idx, val);
    }

#if THE_ISA == MIPS_ISA
    RegVal
    readRegOtherThread(const RegId& misc_reg, ThreadID tid)
    {
        panic("MIPS MT not defined for O3 CPU.\n");
        return 0;
    }

    void
    setRegOtherThread(const RegId& misc_reg, RegVal val, ThreadID tid)
    {
        panic("MIPS MT not defined for O3 CPU.\n");
    }
#endif
};

#endif // __CPU_O3_ALPHA_DYN_INST_HH__
