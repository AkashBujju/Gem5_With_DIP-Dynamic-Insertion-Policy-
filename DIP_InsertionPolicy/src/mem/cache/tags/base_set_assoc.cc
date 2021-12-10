/*
 * Copyright (c) 2012-2014 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
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

/**
 * @file
 * Definitions of a conventional tag store.
 */

#include "mem/cache/tags/base_set_assoc.hh"

#include <string>

#include "base/intmath.hh"

	BaseSetAssoc::BaseSetAssoc(const Params *p)
:BaseTags(p), allocAssoc(p->assoc), blks(p->size / p->block_size),
	sequentialAccess(p->sequential_access),
	replacementPolicy(p->replacement_policy)
{
	// Check parameters
	if (blkSize < 4 || !isPowerOf2(blkSize)) {
		fatal("Block size must be at least 4 and a power of 2");
	}
}

	void
BaseSetAssoc::tagsInit(std::string cache_name)
{
	// Initialize all blocks
	for (unsigned blk_index = 0; blk_index < numBlocks; blk_index++) {
		// Locate next cache block
		CacheBlk* blk = &blks[blk_index];

		// Link block to indexing policy
		indexingPolicy->setEntry(blk, blk_index);

		// Associate a data chunk to the block
		blk->data = &dataBlks[blkSize*blk_index];

		// Associate a replacement data entry to the block
		blk->replacementData = replacementPolicy->instantiateEntry();
	}

	/* @Akash */
	this->cache_name = cache_name;

	{
		unsigned int K = 32;
		unsigned int N = indexingPolicy->getNumSets();
		int num_bits_constituency = log2(K);
		int num_bits_set = log2(N / K);

		/* Initialising the set_belongs_to* map */
		for(int set_number = 0; set_number < indexingPolicy->getNumSets(); ++set_number) {
			set_belongs_to_lru[set_number] = false;
			set_belongs_to_bip[set_number] = false;
		}
		/* Initialising the set_belongs_to* map */

		/* Initialising the num_accesses map */
		for(int set_number = 0; set_number < indexingPolicy->getNumSets(); ++set_number) {
			for(unsigned int j = 0; j < indexingPolicy->getAssoc(); ++j) {
				std::string key = std::to_string(set_number) + "_" + std::to_string(j);
			}
		}
		/* Initialising the num_accesses map */

		for(int current_constituency = 0; current_constituency < K; ++current_constituency) {
			for(int current_set = 0; current_set < (N / K); ++current_set) {
				int diff = abs(num_bits_set - num_bits_constituency);
				if(num_bits_set > num_bits_constituency) {
					unsigned int B1 = current_constituency << (num_bits_set - diff);
					unsigned int B2 = current_set >> diff;
					unsigned int set_number = B1 | B2;
					unsigned int complement = ((1 << (num_bits_set - diff)) - 1) & (~(current_set >> diff));

					if(current_constituency == (current_set >> diff))
						set_belongs_to_lru[set_number] = true;
					else if(current_constituency == complement)
						set_belongs_to_bip[set_number] = true;

					/*
					if(cache_name == "system.l2") {
						std::ofstream file("system.l2.txt", std::ios::app);
						if(current_constituency == complement)
							file << set_number << "\n";
						file.close();
					}
					*/
				}
				else {
					unsigned int B1 = (current_constituency >> diff) << num_bits_set;
					unsigned int B2 = current_set;
					unsigned int set_number = B1 | B2;
					unsigned int complement = ((1 << num_bits_set) - 1) & (~current_set);
					if((current_constituency >> diff) == current_set)
						set_belongs_to_lru[set_number] = true;
					else if((current_constituency >> diff) == complement)
						set_belongs_to_bip[set_number] = true;

					/*
					if(cache_name == "system.cpu.dcache") {
						std::ofstream file("system.cpu.dcache.txt", std::ios::app);
						if((current_constituency >> diff) == complement)
							file << set_number << "\n";
						file.close();
					}
					*/
				}
			}
		}
	}
	/* @Akash */
}

	void
BaseSetAssoc::invalidate(CacheBlk *blk)
{
	BaseTags::invalidate(blk);

	// Decrease the number of tags in use
	stats.tagsInUse--;

	// Invalidate replacement data
	replacementPolicy->invalidate(blk->replacementData);
}

	BaseSetAssoc *
BaseSetAssocParams::create()
{
	// There must be a indexing policy
	fatal_if(!indexing_policy, "An indexing policy is required");

	return new BaseSetAssoc(this);
}
