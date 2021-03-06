# Gem5_DIP_InsertionPolicy_Hack
### Hacking the GEM5 CPU Simulator to implement the DIP (Dynamic Insertion Policy). Paper by Qureshi, Patt, Joel, Jaleel, Simon Steely.

##### Checkout the project report for a description of the implmentation.

This project implements the DIP (Dynamic Insertion Policy).

All the results for the comparison of LRU (baseline) and DIP can be found in the RESULTS folder. The folder SIMPOINT data contains the simpoint information for each benchmark.

Note: Simpoint itself is not needed to re-run the simulations as we already have the checkpoints generated in the SIMPOINT data folder.

Note: Major files changed are:
   * src/mem/cache/Cache.py
   * src/mem/cache/replacement_policies/bip_rp.hh and bip_rp.cc
   * src/mem/cache/replacement_policies/base.hh
   * src/mem/cache/tags/base_set_assoc.hh

   Search for the work '@Akash' within the src/mem/cache subdirectory to get a list of all files and lines changed/added.

Note: The DIP policy is implmented inside the BIP replacement policy.

Here is an example of how to run the 'bzip2' benchmark that used DIP:
   1) Goto the file src/mem/cache/Cache.py and change the replacement policy to BIPRP.
   2) Goto the file src/mem/cache/tags/base_set_assoc.hh and uncomment the lines 171, 172, 173 for DIP to work. If you want LRU then comment these line and also change the replacement policy to LRURP in Cache.py file.
   3) Run 'scons-3 ./build/X86/gem5.opt -j 4'
   4) Run './build/X86/gem5.opt -d RESULTS/bzip2_DIP configs/spec2k6/run.py -b bzip2 --restore-simpoint-checkpoint -r1 --checkpoint-dir SIMPOINT_DATA/bzip2/ --cpu-type=DerivO3CPU --restore-with-cpu=AtomicSimpleCPU --maxinsts=21000000 --l1d_size=4kB --l1i_size=4kB --caches --l2cache --l2_size=16kB'
