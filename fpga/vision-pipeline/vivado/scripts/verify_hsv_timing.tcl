set script_dir [file dirname [file normalize [info script]]]
set root_dir [file normalize [file join $script_dir ..]]
set checkpoint [file join $root_dir integration hw hw.runs impl_1 \
    system_wrapper_postroute_physopt.dcp]

if {![file exists $checkpoint]} {
    error "Post-route checkpoint does not exist: $checkpoint"
}

open_checkpoint $checkpoint

set hsv_cells [get_cells -hier -quiet -filter \
    {NAME =~ *hsv_blue_classifier_i*}]
set hsv_nets [get_nets -hier -quiet -filter \
    {NAME =~ *hsv_blue_classifier_i*}]
set hsv_target_pins [get_pins -hier -quiet -filter \
    {NAME =~ *hsv_blue_classifier_i*/*/D}]
set stage1_target [get_pins -hier -quiet -filter \
    {NAME =~ *stage1_blue_hit_reg/D}]
set timing_targets [concat $hsv_target_pins $stage1_target]

puts "HSV_VERIFY_CELL_COUNT=[llength $hsv_cells]"
puts "HSV_VERIFY_NET_COUNT=[llength $hsv_nets]"
puts "HSV_VERIFY_TARGET_PIN_COUNT=[llength $timing_targets]"

if {[llength $hsv_cells] == 0 || [llength $timing_targets] == 0} {
    error "HSV classifier hierarchy or timing endpoints are missing"
}

set worst_path [get_timing_paths -quiet -delay_type max \
    -to $timing_targets -max_paths 1 -nworst 1]
if {[llength $worst_path] != 1} {
    error "Could not obtain the HSV worst setup path"
}

puts "HSV_WORST_SETUP_SLACK_NS=[get_property SLACK $worst_path]"
puts "HSV_WORST_SETUP_START=[get_property STARTPOINT_PIN $worst_path]"
puts "HSV_WORST_SETUP_END=[get_property ENDPOINT_PIN $worst_path]"

report_timing -delay_type max -to $timing_targets -max_paths 10 \
    -file [file join $root_dir output hsv_timing_paths.rpt]

close_design
exit
