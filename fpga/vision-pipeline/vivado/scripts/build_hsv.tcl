set script_dir [file dirname [file normalize [info script]]]
set root_dir [file normalize [file join $script_dir ..]]
set hw_dir [file join $root_dir integration hw]
set project_file [file join $hw_dir hw.xpr]
set output_dir [file join $root_dir output]
set hsv_source [file join $root_dir src basic_rgb_hsv_blue_classifier.sv]

file mkdir $output_dir
set_param general.maxThreads 8
set_param board.repoPaths [list [file join $hw_dir hw.board]]

puts "HSV_BUILD_ROOT=$root_dir"
puts "HSV_PROJECT=$project_file"
open_project $project_file

if {[llength [get_files -quiet $hsv_source]] == 0} {
    add_files -norecurse $hsv_source
    set_property file_type SystemVerilog [get_files $hsv_source]
}
update_compile_order -fileset sources_1

set bd_file [get_files -quiet */system.bd]
if {[llength $bd_file] != 1} {
    error "Expected one system.bd, found [llength $bd_file]"
}
open_bd_design $bd_file
set tracker_cell [get_bd_cells -quiet basic_pcam_1080_axis_0]
if {[llength $tracker_cell] != 1} {
    error "basic_pcam_1080_axis_0 module-reference cell not found"
}
# Vivado 2020.2 refreshes unchanged module-reference interfaces from compile
# order when output products are regenerated. The wrapper ports are unchanged.
validate_bd_design
save_bd_design
generate_target all $bd_file

set tracker_run [get_runs -quiet system_basic_pcam_1080_axis_0_0_synth_1]
if {[llength $tracker_run] == 1} {
    reset_run $tracker_run
    launch_runs $tracker_run -jobs 8
    wait_on_run $tracker_run
    set tracker_status [get_property STATUS $tracker_run]
    puts "HSV_TRACKER_RUN_STATUS=$tracker_status"
    if {![string match "*Complete*" $tracker_status]} {
        error "HSV tracker synthesis did not complete"
    }
}

reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
set synth_status [get_property STATUS [get_runs synth_1]]
puts "HSV_SYNTH_STATUS=$synth_status"
if {![string match "*Complete*" $synth_status]} {
    error "Top-level synthesis did not complete"
}

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
set impl_status [get_property STATUS [get_runs impl_1]]
puts "HSV_IMPL_STATUS=$impl_status"
if {![string match "*Complete*" $impl_status]} {
    error "Implementation/bitstream did not complete"
}

set impl_bit [file join $hw_dir hw.runs impl_1 system_wrapper.bit]
set output_bit [file join $output_dir basic_robot_flow_hsv_test.bit]
if {![file exists $impl_bit]} {
    error "Implementation bitstream was not generated: $impl_bit"
}
file copy -force $impl_bit $output_bit

set output_xsa [file join $output_dir basic_robot_flow_hsv_test.xsa]
write_hw_platform -fixed -include_bit -force -file $output_xsa

puts "HSV_BITSTREAM=$output_bit"
puts "HSV_XSA=$output_xsa"
close_project
exit
