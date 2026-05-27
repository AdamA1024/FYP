# dump_results.tcl — pull the buck experiment results buffer out of DDR.
#
# Run from the XSCT Console once buck_ctrl has printed "buck experiments
# complete ..." and parked in its while(1). Your debug session already has a
# target selected, so you normally just need:
#
#   xsct% cd C:/Users/adama/Documents/CIL          ;# where results.bin lands
#   xsct% source dump_results.tcl
#
# Optional args:  source dump_results.tcl ?outfile? ?base? ?words?
#
# Produces results.bin; decode with:  python3 parse_results.py results.bin
proc dump_results {{out results.bin} {base 0x10000000} {words 0x5400}} {
    # If no target is current (e.g. fresh XSCT), grab the first ARM core. With an
    # active debug session this is a no-op — we keep whatever is already selected.
    if {[catch {targets -filter {jtag_cable_name != ""} -target-properties}]} {
        catch { connect }
    }
    catch {
        if {[lindex [targets -current] 0] eq ""} {
            targets -set -filter {name =~ "*Cortex-A*#0" || name =~ "ARM*Cortex-A*0"}
        }
    }

    # Poll DONE_MAGIC at word 4 (byte offset 0x10) so we never dump a half-filled buffer.
    set done_addr [expr {$base + 0x10}]
    set done 0
    for {set tries 0} {$tries < 200} {incr tries} {
        if {[catch {mrd -value $done_addr} v] == 0 && $v == 0xD09ED09E} { set done 1; break }
        after 50
    }
    if {!$done} {
        puts "WARNING: DONE_MAGIC not set — dumping anyway (results may be partial)."
    }

    mrd -bin -file $out $base $words
    puts "wrote $out ([expr {$words*4}] bytes from $base)"
}

# Run once on source with defaults. To re-dump to another file/region, just call
# the proc again, e.g.:  dump_results my.bin 0x10000000 0x5400
dump_results
