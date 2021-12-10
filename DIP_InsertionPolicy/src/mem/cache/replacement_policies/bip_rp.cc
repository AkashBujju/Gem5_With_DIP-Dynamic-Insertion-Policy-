/**
 * Copyright (c) 2018 Inria
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
 */

#include "mem/cache/replacement_policies/bip_rp.hh"

#include <memory>
/* @Akash */
#include <fstream>
/* @Akash */

#include "base/random.hh"
#include "params/BIPRP.hh"

/* @Akash */
unsigned int counter_1 = 0;
unsigned int counter_2 = 0;
/* @Akash */

	BIPRP::BIPRP(const Params *p)
: LRURP(p), btp(p->btp)
{
	replacement_policy_code = 2;
}

BIPRP::~BIPRP()
{
}

void
BIPRP::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
	std::shared_ptr<LRUReplData> casted_replacement_data =
		std::static_pointer_cast<LRUReplData>(replacement_data);
	
	/* @Akash: The code for DIP */
	if(dedicated_repl_mode == 1) { /* Set dedicated to LIP */
		casted_replacement_data->lastTouchTick = 1;
	}
	else if(dedicated_repl_mode == 2) { /* Set dedicated to BIP. */
		if (random_mt.random<unsigned>(1, 100) <= btp) {
			casted_replacement_data->lastTouchTick = curTick();
		} else {
			casted_replacement_data->lastTouchTick = 1;
		}
	}
	else { /* @Akash: Set belongs to neither LRU or BIP. */
		/* @Akash: Use the follower_repl_mode for these sets. */
		if(follower_repl_mode == 1) { /* Use LIP */
			casted_replacement_data->lastTouchTick = 1;
		}
		else if(follower_repl_mode == 2) { /* Use BIP */
			if (random_mt.random<unsigned>(1, 100) <= btp) {
				casted_replacement_data->lastTouchTick = curTick();
			} else {
				casted_replacement_data->lastTouchTick = 1;
			}
		}
	}
	/* @Akash: The code for DIP */

	/* @Akash This was the original BIP code */
	// Entries are inserted as MRU if lower than btp, LRU otherwise
	// if (random_mt.random<unsigned>(1, 100) <= btp) {
	// 	casted_replacement_data->lastTouchTick = curTick();
	// } else {
	// 	// Make their timestamps as old as possible, so that they become LRU
	// 	casted_replacement_data->lastTouchTick = 1;
	// }
}

	BIPRP*
BIPRPParams::create()
{
	return new BIPRP(this);
}
