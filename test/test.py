# SPDX-License-Identifier: Apache-2.0
# Sky TPU 3x3 — cocotb CI test.
# GPIO-mode smoke of the full engine against a Python golden model:
#   * CFG writes over the beat bus
#   * two-tile accumulation band (R=4, Nb=2, shift=2), int4 readout
#   * offset contract: frames at header+6 then every 3 (sampled blind, the
#     way the RP2040 driver will), quiet bus (0x000) off-frame
#   * ReLU re-read of the same band (readout is non-destructive)
#   * STATUS frame at header+3: idle state + band-done sticky
# SPI mode is exercised by the project's full iverilog TB + GL miter; this CI
# test keeps the harness self-contained (no RAM model needed).

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

# ---------------- golden model ----------------
def rq(x, shift, relu):
    h = 0 if shift == 0 else (1 << (shift - 1))
    e = (x + h) >> shift            # python >> floors: matches RTL floor semantics
    e = max(-8, min(7, e))
    if relu and e < 0:
        e = 0
    return e

def clamp15(v):
    return max(-16384, min(16383, v))

def s4(v):
    return v & 0xF

def wfun(st, t, r, c):
    return ((st * 5 + t * 3 + r * 2 + c) % 7) - 3

def afun(st, t, i, lane):
    return ((st * 3 + t * 2 + i * 5 + lane * 7) % 7) - 3

# ---------------- beat helpers ----------------
async def send(dut, seq):
    """Drive bytes back-to-back, one per clock (falling-edge launch), then NOP."""
    for b in seq:
        await FallingEdge(dut.clk)
        dut.ui_in.value = b
    await FallingEdge(dut.clk)
    dut.ui_in.value = 0

async def nops(dut, n):
    await ClockCycles(dut.clk, n)

async def cfg1(dut, cid, v):
    await send(dut, [0x10 | cid, v])

async def load_tile(dut, W):
    seq = [0x20]
    for r in (2, 1, 0):             # bottom row first
        seq += [(s4(W[r][1]) << 4) | s4(W[r][0]), s4(W[r][2])]
    await send(dut, seq)
    await nops(dut, 10)             # host contract H2b

async def run_pass(dut, A):
    seq = [0x30]
    for row in A:
        seq += [(s4(row[1]) << 4) | s4(row[0]), s4(row[2])]
    await send(dut, seq)
    await nops(dut, 25)             # covers DRAIN (14) + margin

def frame(dut):
    return ((int(dut.uio_out.value) >> 4) << 8) | int(dut.uo_out.value)

async def read_frames_at(dut, header, first_off, stride, count):
    """Send header, then sample the pin frame blind at the offset contract
    (FallingEdge sampling: frame_q is registered, stable mid-cycle)."""
    await FallingEdge(dut.clk)
    dut.ui_in.value = header
    await FallingEdge(dut.clk)      # header consumed on the posedge in between
    dut.ui_in.value = 0
    out, quiet_ok = [], True
    for k in range(first_off + stride * (count - 1) + 2):
        await FallingEdge(dut.clk)
        f = frame(dut)
        idx = k + 2                 # falling edges elapsed since header launch
        if idx >= first_off and (idx - first_off) % stride == 0 and \
           (idx - first_off) // stride < count:
            out.append(f)
        elif f != 0:
            quiet_ok = False
    return out, quiet_ok


@cocotb.test()
async def test_gpio_band(dut):
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    R, NB, SHIFT = 4, 2, 2
    await cfg1(dut, 0x0, 0x03)      # a_signed | w_signed
    await cfg1(dut, 0x2, SHIFT)
    await cfg1(dut, 0x3, R)
    await cfg1(dut, 0x4, NB)

    # ---- band: two tiles, accumulated ----
    C = [[0] * 3 for _ in range(R)]
    for t in range(NB):
        W = [[wfun(0, t, r, c) for c in range(3)] for r in range(3)]
        A = [[afun(0, t, i, l) for l in range(3)] for i in range(R)]
        await load_tile(dut, W)
        await run_pass(dut, A)
        for i in range(R):
            for c in range(3):
                acc = sum(A[i][l] * W[l][c] for l in range(3))
                C[i][c] = clamp15(acc if t == 0 else C[i][c] + acc)

    # ---- int4 readout at the offset contract (first=6, stride=3) ----
    frames, quiet = await read_frames_at(dut, 0x40, 6, 3, R)
    exp = [(s4(rq(C[i][2], SHIFT, 0)) << 8) |
           (s4(rq(C[i][1], SHIFT, 0)) << 4) |
            s4(rq(C[i][0], SHIFT, 0)) for i in range(R)]
    assert frames == exp, f"int4 frames {frames} != golden {exp}"
    assert quiet, "bus not quiet between frames (D7-1)"
    dut._log.info(f"int4 band OK at offset contract: {[hex(f) for f in frames]}")

    # ---- ReLU re-read (non-destructive readout) ----
    await cfg1(dut, 0x0, 0x07)      # + relu_en
    frames, _ = await read_frames_at(dut, 0x40, 6, 3, R)
    exp = [(s4(rq(C[i][2], SHIFT, 1)) << 8) |
           (s4(rq(C[i][1], SHIFT, 1)) << 4) |
            s4(rq(C[i][0], SHIFT, 1)) for i in range(R)]
    assert frames == exp, f"relu frames {frames} != golden {exp}"
    dut._log.info("relu re-read OK")

    # ---- STATUS at +3: idle, band-done sticky, no overflow ----
    frames, _ = await read_frames_at(dut, 0x50, 3, 1, 1)
    st = frames[0]
    assert (st & 0x7) == 0, f"STATUS state {st & 0x7} != IDLE"
    assert (st >> 9) & 1 == 1, "STATUS band_done sticky not set"
    assert (st >> 4) & 0x7 == 0, "STATUS ovf_agg not clean"
    dut._log.info(f"STATUS OK: {hex(st)}")
