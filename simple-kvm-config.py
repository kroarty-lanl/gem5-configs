# Copyright (c) 2021 The Regents of the University of California.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
Script to run NAS parallel benchmarks with gem5. The script expects the
benchmark program to run. The input is in the format
<benchmark_prog>.<class>.x .The system is fixed with 2 CPU cores, MESI
Two Level system cache and 3 GB DDR4 memory. It uses the x86 board.

This script will count the total number of instructions executed
in the ROI. It also tracks how much wallclock and simulated time.

Usage:
------

```
scons build/X86/gem5.opt
./build/X86/gem5.opt \
    configs/example/gem5_library/x86-npb-benchmarks.py \
    --benchmark <benchmark_name> \
    --size <benchmark_class>
```
"""

import argparse
import time

import m5
from m5.objects import Root

from gem5.utils.requires import requires
from gem5.components.boards.x86_board import X86Board
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.coherence_protocol import CoherenceProtocol
from gem5.resources.resource import Resource, DiskImageResource
from gem5.simulate.simulator import Simulator
from gem5.simulate.simulator import ExitEvent

from m5.stats.gem5stats import get_simstat
from m5.util import warn

from pathlib import Path

requires(
    isa_required=ISA.X86,
    coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
    kvm_required=True,
)

parser = argparse.ArgumentParser(
    description="An example configuration script to run the npb benchmarks."
)

parser.add_argument(
    "--image",
    type=str,
    required=True,
    help="Path to your disk image",
)

parser.add_argument(
    "--command",
    type=str,
    required=True,
    help="Command to run in simulation",
)

args = parser.parse_args()

from gem5.components.cachehierarchies.classic.no_cache import NoCache

from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)

#cache_hierarchy = MESITwoLevelCacheHierarchy(
#    l1d_size="32kB",
#    l1d_assoc=8,
#    l1i_size="32kB",
#    l1i_assoc=8,
#    l2_size="256kB",
#    l2_assoc=16,
#    num_l2_banks=2,
#)
memory = DualChannelDDR4_2400(size="3GB")

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.KVM,
    switch_core_type=CPUTypes.ATOMIC,
    isa=ISA.X86,
    num_cores=2,
)

board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=NoCache(),
)

board.set_kernel_disk_workload(
    # The x86 linux kernel will be automatically downloaded to the
    # `~/.cache/gem5` directory if not already present.
    # npb benchamarks was tested with kernel version 4.19.83
    kernel=Resource("x86-linux-kernel-4.19.83"),
    # The x86-npb image will be automatically downloaded to the
    # `~/.cache/gem5` directory if not already present.
    disk_image=DiskImageResource(args.image, root_partition="1"),
    readfile_contents=f"m5 exit; {args.command};",
)

simulator = Simulator(
    board=board,
    full_system=True,
    on_exit_event={
        ExitEvent.EXIT: (func() for func in [processor.switch]),
    },
)

print("Running the simulation")
simulator.run()

print(f"Done with the simulation @{simulator.get_current_tick()}")
