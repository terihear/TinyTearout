# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles
from cocotb.triggers import RisingEdge
import random

# =============================================================================
# DUT DRIVER — 3-Cycle Protocol
# =============================================================================

class MultDriver:
    """
    Drives the 3-cycle input protocol:
      Cycle 0: ui_in = coefficient (Q0.7 signed, 8-bit 2's comp)
      Cycle 1: ui_in = multiplicand[7:0]
      Cycle 2: ui_in = multiplicand[15:8]
    Then waits for 2-byte serialized output (LSB first).
    """

    def __init__(self, dut):
        self.dut = dut

    async def reset(self):
        self.dut.rst_n.value = 0
        self.dut.ena.value = 0
        self.dut.ui_in.value = 0
        for _ in range(5):
            await RisingEdge(self.dut.clk)
        self.dut.rst_n.value = 1
        await RisingEdge(self.dut.clk)

    async def send_operand(self, mcand_s16: int, coeff_q07: int):
        """Send one multiply operation using the clean 3-cycle protocol."""
        self.dut.ena.value = 1

        # Cycle 0: Coefficient
        self.dut.ui_in.value = coeff_q07 & 0xFF
        await RisingEdge(self.dut.clk)

        # Cycle 1: Multiplicand low byte
        self.dut.ui_in.value = mcand_s16 & 0xFF
        await RisingEdge(self.dut.clk)

        # Cycle 2: Multiplicand high byte (completes operand load)
        mcand_hi = (mcand_s16 >> 8) & 0xFF
        self.dut.ui_in.value = mcand_hi
        await RisingEdge(self.dut.clk)

        # Release enable after operand is fully loaded
        self.dut.ena.value = 0
        self.dut.ui_in.value = 0

    async def collect_result(self, timeout_cycles=40):
        """Collect 2-byte serialized output (LSB first), return signed 16-bit."""
        lo_byte = None
        got_lo = False

        for _ in range(timeout_cycles):
            await RisingEdge(self.dut.clk)
            val = int(self.dut.uo_out.value)
            avail = int(self.dut.uio_out.value);
            if avail > 0:
                # output LO available
                if ( (not got_lo) and (avail == 1) ):
                    lo_byte = val
                    got_lo = True
                else:
                    hi_byte = val
                    raw = (hi_byte << 8) | lo_byte
                    return raw if raw < 0x8000 else raw - 0x10000

        raise TimeoutError(f"No complete output within {timeout_cycles} cycles")

    async def send_and_collect(self, mcand_s16: int, coeff_q07: int,
                                timeout_cycles=40):
        """Convenience: send operand and return result."""
        await self.send_operand(mcand_s16, coeff_q07)
        # Allow pipeline to process; first output byte appears ~16 cycles later
        # We need to distinguish "no output yet" from "output is zero"
        # Bad Strategy: wait a minimum number of cycles, then collect
        return await self.collect_result(timeout_cycles)

def golden_multiply(mcand_s16: int, coeff_q07: int) -> int:
    """
    Exact Python model matching RTL: Q0.7 fixed-point multiply with
    round-half-up, clamped to 16-bit signed.
    """
    coeff_signed = coeff_q07 if coeff_q07 < 128 else coeff_q07 - 256
    exact = mcand_s16 * coeff_signed

    # Round half-up (matches RTL acc[6] check)
    if exact >= 0:
        rounded = (exact + 64) >> 7
    else:
        rounded = -((-exact + 64) >> 7)

    return max(-32768, min(32767, rounded))


@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    # Set the clock period to 10 us (100 KHz)
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("Reset")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    dut._log.info("Test project behavior")

# =============================================================================
# LATENCY FALSIFICATION
# =============================================================================

@cocotb.test()
async def test_latency_falsification(dut):
    """
    Measure actual latency from mcand-HI-presented to product-LSB-emitted.
    Spec: 16 cycles. Attempt to falsify with 50 random trials.
    Set the sim clock period to 20 ns (50 MHz)
    """
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())
    drv = MultDriver(dut)
    await drv.reset()

    NUM_TRIALS = 50
    latencies = []

    for trial in range(NUM_TRIALS):
        mcand = random.randint(-32768, 32767)
        coeff = random.randint(-128, 127)

        # Manually drive to get precise timestamps
        dut.ena.value = 1

        # Cycle 0: coeff
        dut.ui_in.value = coeff & 0xFF
        await RisingEdge(dut.clk)

        # Cycle 1: mcand_lo
        dut.ui_in.value = mcand & 0xFF
        await RisingEdge(dut.clk)

        # Cycle 2: mcand_hi ← REFERENCE POINT (mcand fully presented)
        dut.ui_in.value = (mcand >> 8) & 0xFF
        t_start = cocotb.utils.get_sim_time(unit="us")
        await RisingEdge(dut.clk)

        dut.ena.value = 0
        dut.ui_in.value = 0

        # Wait for first output byte
        await self.collect_result(40)
        t_end = cocotb.utils.get_sim_time(unit="us")
        latency = round((t_end - t_start) / 10.0)
        latencies.append(latency)

    unique = sorted(set(latencies))
    dut._log.info(f"Latency over {NUM_TRIALS} trials: unique={unique}, "
                  f"min={min(latencies)}, max={max(latencies)}")

# =============================================================================
# BOUNDARY COEFFICIENTS
# =============================================================================

BOUNDARY_CASES = [
    ("max_pos_coeff",        16384,  127, "+127/128 × 16384"),
    ("max_pos_neg_mcand",   -16384,  127, "+127/128 × -16384"),
    ("neg_one",              16384,  128, "-1.0 × 16384"),
    ("neg_one_neg_mcand",   -16384,  128, "-1.0 × -16384"),
    ("zero_coeff",           32000,    0, "0 × 32000"),
    ("zero_coeff_neg",      -32000,    0, "0 × -32000"),
    ("min_neg_coeff",        16384,  255, "-1/128 × 16384"),
    ("min_neg_coeff_neg",   -16384,  255, "-1/128 × -16384"),
    ("near_max_pos",         32767,  127, "+127/128 × 32767"),
    ("neg_unity_boundary",  -32768,  128, "-1.0 × -32768"),
    ("small_pos",                1,  127, "+127/128 × 1"),
    ("small_neg_frac",          64,  255, "-1/128 × 64 = -0.5 → rounds to 0"),
]


@cocotb.test()
async def test_boundary_coefficients(dut):
    """Verify correct results for all boundary coefficient values."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    drv = MultDriver(dut)
    await drv.reset()

    failures = []
    for name, mcand, coeff, desc in BOUNDARY_CASES:
        expected = golden_multiply(mcand, coeff)
        actual = await drv.send_and_collect(mcand, coeff)

        ok = actual == expected
        tag = "PASS" if ok else "FAIL"
        dut._log.info(f"[{tag}] {name}: {desc} | exp={expected} act={actual}")
        if not ok:
            failures.append(f"  {name}: expected={expected}, got={actual}")

    assert not failures, "Boundary failures:\n" + "\n".join(failures)


# =============================================================================
# ROUNDING HALF-UP
# =============================================================================

def generate_half_round_cases(n=30):
    """Find (mcand, coeff) where |mcand*coeff| mod 128 == 64."""
    cases = []
    attempts = 0
    while len(cases) < n and attempts < 200000:
        attempts += 1
        m = random.randint(-32768, 32767)
        c = random.randint(-128, 127)
        prod = m * c
        if abs(prod) % 128 == 64:
            cases.append((m, c))
    return cases


@cocotb.test()
async def test_rounding_half_up(dut):
    """Verify round-half-up when fractional residue is exactly 0.5."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    drv = MultDriver(dut)
    await drv.reset()

    cases = generate_half_round_cases(30)
    assert len(cases) >= 15, f"Only found {len(cases)} half-round cases"

    failures = []
    for mcand, coeff in cases:
        expected = golden_multiply(mcand, coeff)
        actual = await drv.send_and_collect(mcand, coeff)

        ok = actual == expected
        dut._log.info(
            f"[{'PASS' if ok else 'FAIL'}] Round: {mcand}×{coeff}/128 "
            f"→ exp={expected} act={actual}"
        )
        if not ok:
            failures.append(f"  {mcand}×{coeff}: exp={expected}, got={actual}")

    assert not failures, "Rounding failures:\n" + "\n".join(failures)


# =============================================================================
# OVERFLOW FALSIFICATION
# =============================================================================

@cocotb.test()
async def test_overflow_falsification(dut):
    """Exhaustive corner sweep + random stress to falsify overflow safety."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    drv = MultDriver(dut)
    await drv.reset()

    corners_m = [-32768, -32767, -1, 0, 1, 32766, 32767]
    corners_c = [-128, -127, -1, 0, 1, 126, 127]

    failures = []
    count = 0

    # Corner sweep
    for m in corners_m:
        for c in corners_c:
            count += 1
            expected = golden_multiply(m, c)

            # Verify golden model doesn't clip (spec violation)
            cs = c if c < 128 else c - 256
            exact = m * cs
            unclamped = (exact + 64) >> 7 if exact >= 0 else -((-exact + 64) >> 7)
            if unclamped != max(-32768, min(32767, unclamped)):
                failures.append(f"SPEC VIOLATION: {m}×{c}/128={unclamped} overflows!")

            actual = await drv.send_and_collect(m, c)
            if actual != expected:
                failures.append(f"  Corner {m}×{c}: exp={expected}, got={actual}")

    # Random stress
    for _ in range(500):
        count += 1
        m = random.randint(-32768, 32767)
        c = random.randint(-128, 127)
        expected = golden_multiply(m, c)
        actual = await drv.send_and_collect(m, c)
        if actual != expected:
            failures.append(f"  Stress {m}×{c}: exp={expected}, got={actual}")

    dut._log.info(f"Overflow falsification: {count} tests completed")
    assert not failures, f"Errors ({len(failures)}):\n" + "\n".join(failures[:20])
