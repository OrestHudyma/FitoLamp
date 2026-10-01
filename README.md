# FitoLamp

## Global Alarm

The PSoC1 slave accepts `$SHGLB,ALARM,*01` followed by LF. It has no device ID:
every lamp running this firmware can react to the same broadcast. Addressed
`SHFTL,ALARM,<id>` is no longer handled. Existing addressed ON/OFF/FON/FOFF
commands (including lamp-group ID 0) retain their behavior.

Alarm performs 20 fade-out/fade-in cycles, restores the previous target brightness
(including OFF or an in-progress ramp), and enables the existing three-hour
manual override. It is an effect, not a safety alarm or persistent alarm mode.
DAY/NIGHT remain ignored by this slave; receiving global Alarm does not implicitly
add other global behaviors. Radio duplicates can repeat the effect. The effect
blocks foreground command processing, so OFF is not immediate cancellation.

RF packets are copied with the RF interrupt briefly masked before dispatch;
PWM interrupts remain enabled during the effect. The shared 16-bit power target
is written with its PWM interrupt masked. Header matching uses immutable full
headers and the comma delimiter, rather than a mutable previous packet.
The existing parser does not validate RF checksums; this change does not claim
to harden the whole protocol or add reliable delivery.

## Verification and deployment

Run `python test.py -v` with GCC installed (or set `CC` to its executable).
Host tests compile the actual main.c with stubbed PSoC APIs. They test RF byte
reception, global dispatch, legacy rejection, addressed power, snapshot
interleaving and brightness restoration. They do not validate MCU timing,
interrupt latency, memory layout or RF reception. CI also uses ASan/UBSan.

On 2026-10-01 all 13 host tests passed. The new main.c also compiled to an
object with the installed ICCM8C compiler, CY8C28045 headers and the original
checkout's generated User Module headers (LMM configuration). This was a
translation-unit check, not a complete regenerated firmware link or hardware test.
Use the project's flash-aware comparison helper for constant headers: this
compiler's standard strncmp does not accept flash/const strings.

Build/regenerate the project in **PSoC Designer 5.4** for **CY8C28445** before
flashing. This is not the PSoC Creator USB/433 interface project. Generated
libraries and build outputs are not included in this PR. No device is flashed
automatically. Upgrade the lamp before using Bridge 0.1.4 Global controls Alarm,
then verify the effect from both ON and OFF and update old HA button references.

The change was prepared in a separate worktree because the original checkout
contains uncommitted project/LED/build changes. Those remain untouched. The
user's local Alarm effect and RX-header correction are incorporated here, without
including unrelated generated outputs or LED/project configuration changes.
