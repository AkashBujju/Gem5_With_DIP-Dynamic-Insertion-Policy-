/*
 * Copyright (c) 2012-2014,2017 ARM Limited
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
 * Declaration of a base set associative tag store.
 */

#ifndef __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
#define __MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/indexing_policies/base.hh"
#include "mem/cache/tags/indexing_policies/set_associative.hh"
#include "mem/packet.hh"
#include "params/BaseSetAssoc.hh"

/* @Akash */
// #include <fstream>
#include <cmath> // @MAYBE TMP
#include <bitset> // @TMP
#include <map>
/* @Akash */

/**
 * A basic cache tag store.
 * @sa  \ref gem5MemorySystem "gem5 Memory System"
 *
 * The BaseSetAssoc placement policy divides the cache into s sets of w
 * cache lines (ways).
 */
class BaseSetAssoc : public BaseTags
{
	protected:
		/** The allocatable associativity of the cache (alloc mask). */
		unsigned allocAssoc;

		/** The cache blocks. */
		std::vector<CacheBlk> blks;

		/** Whether tags and data are accessed sequentially. */
		const bool sequentialAccess;

		/** Replacement policy */
		BaseReplacementPolicy *replacementPolicy;

	public:
		/** Convenience typedef. */
		typedef BaseSetAssocParams Params;

		/* @Akash */
		unsigned int missCount = 0;
		unsigned int hitCount = 0;
		unsigned int numMisses_LRU = 0;
		unsigned int numMisses_BIP = 0;
		unsigned int PSEL = 0; /* @Akash 10-bit PSEL counter */
		unsigned int max_PSEL = 0; /* @Akash TMP */
		std::map<unsigned int, bool> set_belongs_to_lru;
		std::map<unsigned int, bool> set_belongs_to_bip;
		std::map<std::string, unsigned int> num_access;
		/* @Akash */

		/**
		 * Construct and initialize this tag store.
		 */
		BaseSetAssoc(const Params *p);

		/**
		 * Destructor
		 */
		virtual ~BaseSetAssoc() {};

		/**
		 * Initialize blocks as CacheBlk instances.
		 */
		void tagsInit(std::string cache_name) override;

		/**
		 * This function updates the tags when a block is invalidated. It also
		 * updates the replacement data.
		 *
		 * @param blk The block to invalidate.
		 */
		void invalidate(CacheBlk *blk) override;

		/* @Akash:
		 * return 0: neither lru or bip.
		 * return 1: set dedicated to lru.
		 * return 2: set dedicated to bip.
		 */
		int is_set_bip_or_lru(Addr set)
		{
			if(set_belongs_to_lru[set]) return 1;
			else if(set_belongs_to_bip[set]) return 2;
			
			return 0;
		}

		/**
		 * Access block and update replacement data. May not succeed, in which case
		 * nullptr is returned. This has all the implications of a cache access and
		 * should only be used as such. Returns the tag lookup latency as a side
		 * effect.
		 *
		 * @param addr The address to find.
		 * @param is_secure True if the target memory space is secure.
		 * @param lat The latency of the tag lookup.
		 * @return Pointer to the cache block if found.
		 */
		CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat) override
		{
			CacheBlk *blk = findBlock(addr, is_secure);

			if(blk != nullptr) {
				std::string key = std::to_string(blk->getSet()) + "_" + std::to_string(blk->getWay());
				num_access[key] += 1;

				// std::ofstream file("numAccess_" + cache_name + ".txt", std::ios::app);
				// file << std::to_string(blk->getSet()) + "_" + std::to_string(blk->getWay()) + ": " + std::to_string(num_access[key]) + "\n";
				// file.close();

				/* @Akash: Uncomment this for LIP and BIP */
				if((replacementPolicy->replacement_policy_code == 1 || replacementPolicy->replacement_policy_code == 2)
					&& num_access[key] > 1) {
					replacementPolicy->touch(blk->replacementData);
				}
			}
			
			/* @Akash */
			// std::string filename = "test_" + cache_name + ".txt";
			unsigned int result = is_set_bip_or_lru(addr);
			if(blk == nullptr) {
				if(result == 1) {
					numMisses_LRU += 1;
					if(PSEL < 1024) PSEL += 1;
				}
				else if(result == 2) {
					numMisses_BIP += 1;
					if(PSEL > 0) PSEL -= 1;
				}
			}

			/* @Akash Setting replacementPolicy->follower_repl_mode based on the PSEL counter. */
			if(PSEL >= 1024) 
				replacementPolicy->follower_repl_mode = 2;
			else				  
				replacementPolicy->follower_repl_mode = 1;
			
			// @Akash: Printing out the max PSEL value to file.
			{
				// if(PSEL > max_PSEL)
				// 	max_PSEL = PSEL;
				// std::ofstream file("PSEL_values.txt");
				// file << std::to_string(max_PSEL) << "\n";
				// file.close();
			}
			/* @Akash Setting replacementPolicy->follower_repl_mode based on the PSEL counter. */

			// std::ofstream file(filename);
			// file << "LRU: " << std::to_string(numMisses_LRU) + ", BIP: " + std::to_string(numMisses_BIP) + ", total: " << std::to_string(missCount) << "\n";;
			// file.close();
			/* @Akash */

			// Access all tags in parallel, hence one in each way.  The data side
			// either accesses all blocks in parallel, or one block sequentially on
			// a hit.  Sequential access with a miss doesn't access data.
			stats.tagAccesses += allocAssoc;
			if (sequentialAccess) {
				if (blk != nullptr) {
					stats.dataAccesses += 1;
				}
			} else {
				stats.dataAccesses += allocAssoc;
			}

			// If a cache hit
			if (blk != nullptr) {
				// Update number of references to accessed block
				blk->refCount++;

				// Update replacement data of accessed block
				replacementPolicy->touch(blk->replacementData);

				/* @Akash */
				hitCount += 1;
				// std::string filename = "cache_info_hits_" + cache_name + ".txt";
				// std::ofstream file(filename);
				// file << hitCount << "\n";
				// file.close();
				/* @Akash */
			}
			else {
				/* @Akash */
				missCount += 1;
				// std::string filename = "cache_info_misses_" + cache_name + ".txt";
				// std::ofstream file(filename);
				// file << missCount << "\n";
				// file.close();
				/* @Akash */
			}

			// The tag lookup latency is the same for a hit or a miss
			lat = lookupLatency;

			return blk;
		}

		/**
		 * Find replacement victim based on address. The list of evicted blocks
		 * only contains the victim.
		 *
		 * @param addr Address to find a victim for.
		 * @param is_secure True if the target memory space is secure.
		 * @param size Size, in bits, of new block to allocate.
		 * @param evict_blks Cache blocks to be evicted.
		 * @return Cache block to be replaced.
		 */
		CacheBlk* findVictim(Addr addr, const bool is_secure,
				const std::size_t size,
				std::vector<CacheBlk*>& evict_blks) override
		{
			// Get possible entries to be victimized
			const std::vector<ReplaceableEntry*> entries =
				indexingPolicy->getPossibleEntries(addr);

			// Choose replacement victim from replacement candidates
			CacheBlk* victim = static_cast<CacheBlk*>(replacementPolicy->getVictim(
						entries));

			// There is only one eviction for this replacement
			evict_blks.push_back(victim);

			return victim;
		}

		/**
		 * Insert the new block into the cache and update replacement data.
		 *
		 * @param pkt Packet holding the address to update
		 * @param blk The block to update.
		 */
		void insertBlock(const PacketPtr pkt, CacheBlk *blk) override
		{
			// Insert block
			BaseTags::insertBlock(pkt, blk);

			// Increment tag counter
			stats.tagsInUse++;

			/* @Akash */
			std::string key = std::to_string(blk->getSet()) + "_" + std::to_string(blk->getWay());
			num_access[key] += 1;

			// @Akash: Finding which dedicated replacement mode to use when resetting. */
			replacementPolicy->dedicated_repl_mode = is_set_bip_or_lru(blk->getSet());
			/* @Akash */

			// std::ofstream file("insertBlock.txt", std::ios::app);
			// file << blk->getSet() << ", " << blk->getWay() << "\n";
			// file.close();

			// Update replacement policy
			replacementPolicy->reset(blk->replacementData);
		}

		/**
		 * Limit the allocation for the cache ways.
		 * @param ways The maximum number of ways available for replacement.
		 */
		virtual void setWayAllocationMax(int ways) override
		{
			fatal_if(ways < 1, "Allocation limit must be greater than zero");
			allocAssoc = ways;
		}

		/**
		 * Get the way allocation mask limit.
		 * @return The maximum number of ways available for replacement.
		 */
		virtual int getWayAllocationMax() const override
		{
			return allocAssoc;
		}

		/**
		 * Regenerate the block address from the tag and indexing location.
		 *
		 * @param block The block.
		 * @return the block address.
		 */
		Addr regenerateBlkAddr(const CacheBlk* blk) const override
		{
			return indexingPolicy->regenerateAddr(blk->tag, blk);
		}

		void forEachBlk(std::function<void(CacheBlk &)> visitor) override {
			for (CacheBlk& blk : blks) {
				visitor(blk);
			}
		}

		bool anyBlk(std::function<bool(CacheBlk &)> visitor) override {
			for (CacheBlk& blk : blks) {
				if (visitor(blk)) {
					return true;
				}
			}
			return false;
		}
};

#endif //__MEM_CACHE_TAGS_BASE_SET_ASSOC_HH__
