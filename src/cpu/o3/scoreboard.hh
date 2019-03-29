/*
 * Copyright (c) 2005-2006 The Regents of The University of Michigan
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
 * Authors: Korey Sewell
 *          Kevin Lim
 *          Steve Reinhardt
 */

#ifndef __CPU_O3_SCOREBOARD_HH__
#define __CPU_O3_SCOREBOARD_HH__

#include <iostream>
#include <utility>
#include <vector>

#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/o3/comm.hh"
#include "debug/Scoreboard.hh"

/**
 * Implements a simple scoreboard to track which registers are
 * ready. This class operates on the unified physical register space,
 * because the different classes of registers do not need to be distinguished.
 * Registers being part of a fixed mapping are always considered ready.
 */
class Scoreboard
{
  private:
    /** The object name, for DPRINTF.  We have to declare this
     *  explicitly because Scoreboard is not a SimObject. */
    const std::string _name;

    /** Scoreboard of physical integer registers, saying whether or not they
     *  are ready. */
    std::vector<bool> regScoreBoard;
        std::vector<int> ref_cnt;
    /** The number of actual physical registers */
    unsigned M5_CLASS_VAR_USED numPhysRegs;

  public:
    /** Constructs a scoreboard.
     *  @param _numPhysicalRegs Number of physical registers.
     *  @param _numMiscRegs Number of miscellaneous registers.
     */
    Scoreboard(const std::string &_my_name,
               unsigned _numPhysicalRegs);

    /** Destructor. */
    ~Scoreboard() {}

    /** Returns the name of the scoreboard. */
    std::string name() const { return _name; };

    /** Checks if the register is ready. */
    bool getReg(PhysRegIdPtr phys_reg) const
    {
                DPRINTF(Scoreboard,"getreg phys_reg is %d\n",(long)phys_reg);
        assert((long)phys_reg < numPhysRegs);

        //if (phys_reg->isFixedMapping()) {
            // Fixed mapping regs are always ready
        //    return true;
        //}

        bool ready = regScoreBoard[(long)phys_reg];

        //if (phys_reg->isZeroReg())
        //    assert(ready);
                if ( phys_reg == 0 )
                        return true;
        return ready;
    }

    /** get the register's ref_cnt. */
    int getRegRefCnt(PhysRegIdPtr phys_reg) const
    {
        assert(phys_reg->flatIndex() < numPhysRegs);

        if (phys_reg->isFixedMapping()) {
            // Fixed mapping regs are always ready
            return 1;
        }

        int rc = ref_cnt[phys_reg->flatIndex()];

        if (phys_reg->isZeroReg())
            assert(rc==1);

        return rc;
    }


    /** Sets the register as ready. */
    void setReg(PhysRegIdPtr phys_reg)
    {
                DPRINTF(Scoreboard,"1 setreg phys_reg is %d\n",(long)phys_reg);
        assert( (long)phys_reg < numPhysRegs);

        //if (phys_reg->isFixedMapping()) {
            // Fixed mapping regs are always ready, ignore attempts to change
            // that
        //    return;
        //}

        DPRINTF(Scoreboard, "Setting reg %i as ready\n",
                (long)phys_reg);

        regScoreBoard[(long)phys_reg] = true;
    }

    /** Sets the register as not ready. */
    void unsetReg(PhysRegIdPtr phys_reg)
    {
                DPRINTF(Scoreboard, "get into unset reg func reg id is %d\n",(long)phys_reg);
        //assert(phys_reg->flatIndex() < numPhysRegs);
                assert( (long)phys_reg <numPhysRegs );
        //if (phys_reg->isFixedMapping()) {
            // Fixed mapping regs are always ready, ignore attempts to
            // change that
        //    return;
        //}

        // zero reg should never be marked unready
        //if (phys_reg->isZeroReg())
        //    return;
                if ((long)phys_reg==0){
                        return;
                }
        regScoreBoard[(long)phys_reg] = false;
    }


    /** Sets the register as ready. */
    void setRegRC(PhysRegIdPtr phys_reg,int rcv)
    {
                DPRINTF(Scoreboard,"set reg ref_cnt phys_reg is %d\n",(long)phys_reg);
        //assert(phys_reg->flatIndex() < numPhysRegs);
        assert((long)phys_reg < numPhysRegs);

        //if (phys_reg->isFixedMapping()) {
            // Fixed mapping regs are always ready, ignore attempts to change
            // that
        //    return;
        //}

        DPRINTF(Scoreboard, "Setting reg %i rcv as %i\n",
                (long)phys_reg,/* phys_reg->className(),*/ rcv);

        ref_cnt[(long)phys_reg] = rcv;
    }

    /** Sets the register as not ready. */
    void unsetRegRC(PhysRegIdPtr phys_reg)
    {
                DPRINTF(Scoreboard,"unset reg ref_cnt phys_reg is %d\n",(long)phys_reg);
        //assert(phys_reg->flatIndex() < numPhysRegs);
        assert((long)phys_reg < numPhysRegs);

        //if (phys_reg->isFixedMapping()) {
            // Fixed mapping regs are always ready, ignore attempts to
            // change that
        //    return;
        //}

        // zero reg should never be marked unready
        //if (phys_reg->isZeroReg())
        //    return;
                if ( (long)phys_reg == 0)
                        return;

        ref_cnt[(long)phys_reg] = 0;
    }



    /** add 1 the register's ref_cnt */
    void incrRC(PhysRegIdPtr phys_reg)
    {
        assert((long)phys_reg < numPhysRegs);

        //if (phys_reg->isFixedMapping()) {
            // Fixed mapping regs are always ready, ignore attempts to change
            // that
        //    return;
        //}

        DPRINTF(Scoreboard, "Adding reg %i ref_cnt 1\n",
                phys_reg);

        ref_cnt[(long)phys_reg] += 1;
    }

    /** sub 1 the register's ref_cnt */
    void decrRC(PhysRegIdPtr phys_reg)
    {
        assert((long)phys_reg < numPhysRegs);

        //if (phys_reg->isFixedMapping()) {
            // Fixed mapping regs are always ready, ignore attempts to
            // change that
        //    return;
        //}

        // zero reg should never be marked unready
        //if (phys_reg->isZeroReg())
        //    return;
                if ((long)phys_reg == 0)
                        return;
        ref_cnt[(long)phys_reg] -= 1;
                if (ref_cnt[(long)phys_reg] < 0){
                        ref_cnt[(long)phys_reg] = 0;
                }
    }


};

#endif
