; SPDX-License-Identifier: GPL-3.0-or-later
;
; OpenVRX FX2 hot loops, hand written so their timing is known exactly.
;
; Cycle counts in the comments are FX2 instruction cycles (4 CLKOUT clocks,
; 83.3 ns at 48 MHz) from TRM table 12-2, with the MOVX stretch set to 0
; (CKCON.MD = 000, done in main.c): MOVX = 2 cycles.
; tools/cycles.py recomputes them from the assembler listing.
;
; Every loop returns as soon as the _svc bit is set by an interrupt.
; Calling convention: no arguments, clobbers A, B, R0, R5-R7, DPTR, PSW.

        .module hotloops

        .globl  _loop_auto
        .globl  _loop_watch_hs
        .globl  _pattern_fill_full
        .globl  _pattern_fill_high

        .globl  _svc
        .globl  _watch_use_npak
        .globl  _cnt_ffull
        .globl  _cnt_committed
        .globl  _cnt_skipped
        .globl  _watch_left
        .globl  _watch_reload
        .globl  _pattern_low
        .globl  _pattern_high
        .globl  _pattern_groups

; SFRs
R_MPAGE         = 0x92
R_AUTOPTRH2     = 0x9d
R_AUTOPTRL2     = 0x9e
R_EP2468STAT    = 0xaa
R_EP24FIFOFLGS  = 0xab
R_AUTOPTRSETUP  = 0xaf
; XDATA registers (all in page 0xE6)
X_EP2CS_L       = 0xa3          ; 0xE6A3
X_EP2FIFOBCH_L  = 0xab          ; 0xE6AB
X_INPKTEND_L    = 0x48          ; 0xE648
X_XAUTODAT2     = 0xe67c

        .area   CSEG    (CODE)

; ---------------------------------------------------------------------------
; AUTOIN streaming: the FIFO commits packets by itself; the CPU only counts
; polls that find the EP2 FIFO full (samples dropped while it is full).
; Idle iteration: 10 cycles (0.83 us).
; ---------------------------------------------------------------------------
_loop_auto:
la_top:
        jb      _svc, la_exit                   ; 4
        mov     a, R_EP24FIFOFLGS               ; 2
        rrc     a                               ; 1   C = EP2FF
        jnc     la_top                          ; 3
        inc     _cnt_ffull                      ; 2
        mov     a, _cnt_ffull                   ; 2
        jnz     la_top                          ; 3
        inc     (_cnt_ffull + 1)
        mov     a, (_cnt_ffull + 1)
        jnz     la_top
        inc     (_cnt_ffull + 2)
        mov     a, (_cnt_ffull + 2)
        jnz     la_top
        inc     (_cnt_ffull + 3)
        sjmp    la_top
la_exit:
        ret

; ---------------------------------------------------------------------------
; Watch mode at high speed (512-byte packets, AUTOIN = 0).
;
; A full, not yet committed packet exists when
;     (EP2FIFOBC / 512) - NPAK > 0
; EP2FIFOBC is the total byte count of the slave FIFO and NPAK (EP2CS) the
; number of packets already committed to USB. If the byte count excludes
; committed packets the test is merely conservative (it waits for the USB
; side to drain), so it is safe under either reading of the TRM. NPAK is read
; before the byte count: a packet leaving in between only lowers the result.
; With _watch_use_npak = 0 the byte count alone is used.
;
; Every available packet is committed (INPKTEND = 0x02) or skipped
; (INPKTEND = 0x82, SKIP needs REVCTL.ENH_PKT): commit one, skip N-1.
;
; Cycle budget (typical, counter increments without carry):
;   idle iteration (no packet)        39 cycles  3.25 us
;   iteration that skips a packet     67 cycles  5.58 us
;   iteration that commits a packet   66 cycles  5.50 us
; so at most about 179,000 packets/s (91 MB/s of slave FIFO input) can be
; handled; 13.33 MS/s at 16 bit is 52,083 packets/s (29 % of the budget).
; The first E6xx access after an INPKTEND write is at least 30 cycles later,
; well above the TRM 15.15 synchronisation delay (4 cycles at IFCLK 5 MHz).
; ---------------------------------------------------------------------------
_loop_watch_hs:
        mov     dptr, #0xe6a3                   ; DPH = 0xE6 for the whole loop
lw_top:
        jb      _svc, lw_exit                   ; 4
        mov     a, R_EP24FIFOFLGS               ; 2
        rrc     a                               ; 1   C = EP2FF
        jnc     lw_avail                        ; 3
        inc     _cnt_ffull                      ; 2
        mov     a, _cnt_ffull                   ; 2
        jnz     lw_avail                        ; 3
        inc     (_cnt_ffull + 1)
        mov     a, (_cnt_ffull + 1)
        jnz     lw_avail
        inc     (_cnt_ffull + 2)
        mov     a, (_cnt_ffull + 2)
        jnz     lw_avail
        inc     (_cnt_ffull + 3)
lw_avail:
        clr     a                               ; 1
        jnb     _watch_use_npak, lw_bc          ; 4
        mov     dpl, #X_EP2CS_L                 ; 3
        movx    a, @dptr                        ; 2   EP2CS
        swap    a                               ; 1
        anl     a, #0x07                        ; 2   NPAK
lw_bc:
        mov     r7, a                           ; 1
        mov     dpl, #X_EP2FIFOBCH_L            ; 3
        movx    a, @dptr                        ; 2   EP2FIFOBCH
        clr     c                               ; 1
        rrc     a                               ; 1   full 512-byte packets
        clr     c                               ; 1
        subb    a, r7                           ; 1
        jc      lw_top                          ; 3   none available
        jz      lw_top                          ; 3
        mov     dpl, #X_INPKTEND_L              ; 3
        mov     a, _watch_left                  ; 2
        orl     a, (_watch_left + 1)            ; 2
        jz      lw_commit                       ; 3
        mov     a, #0x82                        ; 2   SKIP | EP2
        movx    @dptr, a                        ; 2
        mov     a, _watch_left                  ; 2   16-bit decrement
        dec     _watch_left                     ; 2
        jnz     lw_skipcnt                      ; 3
        dec     (_watch_left + 1)
lw_skipcnt:
        inc     _cnt_skipped                    ; 2
        mov     a, _cnt_skipped                 ; 2
        jnz     lw_top                          ; 3
        inc     (_cnt_skipped + 1)
        mov     a, (_cnt_skipped + 1)
        jnz     lw_top
        inc     (_cnt_skipped + 2)
        mov     a, (_cnt_skipped + 2)
        jnz     lw_top
        inc     (_cnt_skipped + 3)
        sjmp    lw_top
lw_commit:
        mov     a, #0x02                        ; 2   EP2
        movx    @dptr, a                        ; 2
        mov     _watch_left, _watch_reload      ; 3
        mov     (_watch_left + 1), (_watch_reload + 1) ; 3
        inc     _cnt_committed                  ; 2
        mov     a, _cnt_committed               ; 2
        jnz     lw_top                          ; 3
        inc     (_cnt_committed + 1)
        mov     a, (_cnt_committed + 1)
        jnz     lw_top
        inc     (_cnt_committed + 2)
        mov     a, (_cnt_committed + 2)
        jnz     lw_top
        inc     (_cnt_committed + 3)
        sjmp    lw_top
lw_exit:
        ret

; ---------------------------------------------------------------------------
; Test pattern, complete packet: _pattern_groups * 8 little-endian words
; counting up from _pattern_high:_pattern_low, written through autopointer 2
; into the EP2 buffer currently owned by the CPU. Packets always start at a
; multiple of their own word count, so the low byte can only wrap after the
; last word: the high byte is advanced once, at the end.
; 7 cycles per word: 1,890 cycles (158 us) per 512-byte packet.
; ---------------------------------------------------------------------------
_pattern_fill_full:
        mov     R_AUTOPTRH2, #0xf0              ; EP2FIFOBUF
        mov     R_AUTOPTRL2, #0x00
        mov     R_AUTOPTRSETUP, #0x05           ; APTREN | APTR2INC
        mov     dptr, #X_XAUTODAT2
        mov     r6, _pattern_low
        mov     r7, _pattern_high
        mov     r5, _pattern_groups
pf_group:
        .rept   8
        mov     a, r6                           ; 1
        movx    @dptr, a                        ; 2   low byte
        mov     a, r7                           ; 1
        movx    @dptr, a                        ; 2   high byte
        inc     r6                              ; 1
        .endm
        djnz    r5, pf_group                    ; 3
        mov     a, r6
        jnz     pf_store
        inc     r7
pf_store:
        mov     _pattern_low, r6
        mov     _pattern_high, r7
        ret

; ---------------------------------------------------------------------------
; Test pattern, high bytes only (high speed, opt-in): the buffer still holds
; the low bytes 0..255 written four packets earlier, so only the 256 odd
; bytes are rewritten with the new high byte through MOVX @R0 / MPAGE.
; 4 cycles per byte: about 1,080 cycles (90 us) per packet.
; ---------------------------------------------------------------------------
_pattern_fill_high:
        mov     a, _pattern_high
        mov     R_MPAGE, #0xf0
        mov     r0, #0x01
        mov     r5, #8
ph_lo:
        .rept   16
        movx    @r0, a                          ; 2
        inc     r0                              ; 1
        inc     r0                              ; 1
        .endm
        djnz    r5, ph_lo                       ; 3
        mov     R_MPAGE, #0xf1
        mov     r0, #0x01
        mov     r5, #8
ph_hi:
        .rept   16
        movx    @r0, a
        inc     r0
        inc     r0
        .endm
        djnz    r5, ph_hi
        inc     _pattern_high
        mov     R_MPAGE, #0x00
        ret
