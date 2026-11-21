"""Bind shared fixtures to an explicit historical module filename boundary."""
import hashlib
import json
from pathlib import Path
import re

try:
    from .fixture_binding import FIXED_BASELINE
except ImportError:
    from fixture_binding import FIXED_BASELINE


BASELINE_TILT_HEADER_SHA256 = '1a6bf7f012d2c1b54ff21bb343f492eb85c6c8cd858bb290b9636f234d36d25c'


def fixture_api_flags(revision):
    """Bind the shared fixture API names to the fixed baseline headers."""
    if revision != FIXED_BASELINE:
        return []
    legacy_names = {
        'remote_control_request_t': 'remote_request_t',
        'run_timing_t': 's3_run_timing_t',
        'codec_motion_snapshot_t': 'motion_snapshot_t',
        'integration_codec_control_packet_t': 's3_control_packet_t',
        'integration_codec_status_packet_t': 's3_status_packet_t',
        'imu_sensor_sample_t': 'imu_sample_t',
        'imu_sensor_bus_t': 'imu_bus_t',
        'ir_calibration_begin': 's3_calibration_begin',
        'ir_calibration_add': 's3_calibration_add',
        'ir_calibration_finish': 's3_calibration_finish',
        'ir_calibration_summary': 's3_calibration_summary',
        'ir_calibration_collector_t': 's3_calibration_collector_t',
        'ir_calibration_range_t': 's3_calibration_range_t',
        'ir_calibration_summary_t': 's3_calibration_summary_t',
        'line_detection_defaults': 's3_line_defaults',
        'line_detection_init': 's3_line_init',
        'line_detection_reset': 's3_line_reset',
        'line_detection_update': 's3_line_update',
        'line_detection_t': 's3_line_detector_t',
        'line_detection_observation_t': 's3_line_observation_t',
        'line_detection_config_t': 's3_line_config_t',
        'line_detection_persistence_t': 's3_persistence_t',
        'bc_quiet_add': 's3_quiet_add',
        'bc_quiet_reset': 's3_barcode_quiet_reset',
        'bc_quiet_update': 's3_barcode_quiet_update',
        'bc_quiet_t': 's3_barcode_quiet_t',
        'profile_obstacle_sample_pack': 's5_sample_pack',
        'profile_obstacle_sample_unpack': 's5_sample_unpack',
        'profile_obstacle_build': 's5_profile_build',
        'profile_obstacle_merge_peek': 's5_peek_merge',
        'profile_obstacle_sample_t': 's5_sample_t',
        'profile_obstacle_t': 's5_profile_t',
        'scan_obstacle_t': 's5_scan_t',
        'scan_obstacle_begin': 's5_scan_begin',
        'scan_obstacle_begin_at': 's5_scan_begin_at',
        'scan_obstacle_accept': 's5_scan_accept',
        'scan_obstacle_peek_angle_valid': 's5_peek_angle_valid',
        'line_recovery_t': 's5_recovery_t',
        'line_recovery_reset': 's5_recovery_reset',
        'line_recovery_observe': 's5_recovery_observe',
        'avoidance_planner_route_t': 's5_route_t',
        'avoidance_planner_plan_t': 's5_plan_t',
        'avoidance_planner_build': 's5_plan',
        'codec_motion_pack': 'motion_snapshot_pack',
        'codec_motion_valid': 'motion_snapshot_valid',
        'codec_motion_state': 'motion_snapshot_state',
        'codec_motion_fault': 'motion_snapshot_fault',
        'codec_motion_busy': 'motion_snapshot_busy',
        'codec_motion_speed_valid': 'motion_snapshot_speed_valid',
        'codec_motion_left_mms': 'motion_snapshot_left_mms',
        'codec_motion_right_mms': 'motion_snapshot_right_mms',
        'ctrl_motion_init': 'motion_controller_init',
        'ctrl_motion_step': 'motion_controller_step',
        'ctrl_motion_make_safe': 'motion_controller_make_safe',
        'ctrl_motion_set_velocity': 'motion_controller_set_velocity',
        'ctrl_motion_request_move': 'motion_controller_request_move',
        'ctrl_motion_nav_timeout': 'motion_controller_nav_timeout',
        'ctrl_motion_stop': 'motion_controller_stop',
        'ctrl_motion_cancel': 'motion_controller_cancel',
        'ctrl_motion_reset': 'motion_controller_reset',
        'ctrl_motion_poll': 'motion_controller_poll',
        'ctrl_motion_state': 'motion_controller_state',
        'ctrl_motion_fault': 'motion_controller_fault',
        'ctrl_motion_fault_bits': 'motion_controller_fault_bits',
        'ctrl_motion_command_id': 'motion_controller_command_id',
        'ctrl_motion_ack_complete': 'motion_controller_ack_complete',
        'ctrl_motion_is_stopped': 'motion_controller_is_stopped',
        'ctrl_motion_speed_cap': 'motion_controller_speed_cap',
        'ctrl_motion_setpoint': 'motion_controller_setpoint',
        'ctrl_motion_set_busy': 'motion_controller_set_busy',
        'ctrl_motion_is_busy': 'motion_controller_is_busy',
        'ctrl_motion_report_sink_t': 'motion_report_sink_t',
        'bc_capture_begin_run': 'bcap_begin_run',
        'bc_capture_record': 'bcap_record',
        'bc_capture_end_run': 'bcap_end_run',
        'bc_capture_format_next': 'bcap_format_next',
        'bc_capture_binary_next': 'bcap_binary_next',
        'bc_capture_sample_t': 'bcap_sample_t',
        'bc_capture_episode_t': 'bcap_episode_t',
        'bc_capture_t': 'bcap_t',
        'bc_capture_input_t': 'bcap_input_t',
        'bc_capture_cursor_t': 'bcap_cursor_t',
        'terr_link_init': 'terrain_control_init',
        'terr_link_ready': 'terrain_control_ready',
        'terr_link_reset_ready': 'terrain_control_reset_ready',
        'terr_link_clear_fault': 'terrain_control_clear_fault',
        'terr_link_begin': 'terrain_control_begin',
        'terr_link_poll': 'terrain_control_poll',
        'terr_link_report': 'terrain_control_report',
        'line_navigation_defaults': 's3_navigation_defaults',
        'line_navigation_init': 's3_navigation_init',
        'line_navigation_set_mode': 's3_navigation_set_mode',
        'line_navigation_set_calibration': 's3_navigation_apply_calibration',
        'line_navigation_set_mapping': 's3_navigation_set_mapping',
        'line_navigation_readiness': 's3_navigation_readiness',
        'line_navigation_start': 's3_navigation_start',
        'line_navigation_end_run': 's3_navigation_end_run',
        'line_navigation_reset_fault': 's3_navigation_reset_fault',
        'line_navigation_process': 's3_navigation_process',
        'line_navigation_surface_name': 's3_surface_name',
        'line_navigation_action_name': 's3_action_name',
        'line_navigation_action_t': 's3_nav_action_t',
        'line_navigation_mode_t': 's3_processing_mode_t',
        'line_navigation_run_state_t': 's3_run_state_t',
        'line_navigation_blocker_t': 's3_readiness_flag_t',
        'line_navigation_fault_flag_t': 's3_fault_flag_t',
        'line_navigation_odometry_t': 's3_odometry_sample_t',
        'line_navigation_distance_t': 's3_capture_distance_t',
        'line_navigation_event_flag_t': 's3_event_flag_t',
        'line_navigation_config_t': 's3_navigation_config_t',
        'line_navigation_sensor_t': 's3_processed_sensor_t',
        'line_navigation_output_t': 's3_navigation_output_t',
        'line_navigation_t': 's3_navigation_t',
        'events_motion_classify': 'motion_classify',
        'events_motion_state_t': 'terrain_state_t',
        'hump_detection_debug_t': 'hump_debug_t',
        'hump_detection_reason_t': 'hump_reason_t',
        'hump_detection_t': 'hump_detector_t',
        'hump_detection_sample_quality_t': 'hump_sample_quality_t',
        'terr_data_init': 'terrain_monitor_init',
        'terr_data_reset_run': 'terrain_monitor_reset_run',
        'terr_data_set_geometry': 'terrain_monitor_set_geometry',
        'terr_data_set_mag_cal': 'terrain_monitor_set_mag_cal',
        'terr_data_process': 'terrain_monitor_process',
        'terr_data_pack': 'terrain_pack',
        'terr_data_unpack': 'terrain_unpack',
        'terr_data_hump_geometry_t': 'terrain_hump_geometry_t',
        'terr_data_safety_t': 'terrain_safety_t',
        'terr_data_result_t': 'terrain_result_t',
        'terr_data_odometry_t': 'terrain_odometry_t',
        'terr_data_monitor_t': 'terrain_monitor_t',
        'motion_pack_fault_first': 'motion_fault_first',
        'motion_pack_read_status': 'motion_unpack_status',
        'motion_pack_read_report': 'motion_unpack_report',
        'motion_pack_velocity': 'motion_cmd_velocity',
        'motion_pack_drive': 'motion_cmd_drive',
        'motion_pack_turn': 'motion_cmd_turn',
        'motion_pack_manoeuvre': 'motion_cmd_manoeuvre',
        'motion_pack_stop': 'motion_cmd_stop',
        'motion_pack_reset': 'motion_cmd_reset',
        'motion_pack_decode': 'motion_unpack_cmd',
        'motion_pack_fault_bit': 'motion_fault_bit',
        'motion_pack_cmd_type_t': 'motion_cmd_type_t',
        'motion_pack_cmd_t': 'motion_cmd_t',
        'bc_decoder_defaults': 's3_barcode_defaults',
        'bc_decoder_reset': 's3_barcode_reset',
        'bc_decoder_init': 's3_barcode_init',
        'bc_decoder_decode_runs': 's3_barcode_decode_runs',
        'bc_decoder_update': 's3_barcode_update',
        'bc_decoder_record_reset': 's3_barcode_record_reset',
        'bc_decoder_record_active': 's3_barcode_record_active',
        'bc_decoder_record_runs': 's3_barcode_record_runs',
        'bc_decoder_record_correct_runs': 's3_barcode_record_correct_runs',
        'bc_decoder_record_fix_varying': 's3_barcode_record_fix_varying',
        'bc_decoder_record_update': 's3_barcode_record_update',
        'bc_decoder_config_t': 's3_barcode_config_t',
        'bc_decoder_stored_config_t': 's3_barcode_stored_config_t',
        'bc_decoder_result_t': 's3_barcode_result_t',
        'bc_decoder_stored_result_t': 's3_barcode_stored_result_t',
        'b_is_reversed': 'reversed',
        'bc_decoder_status_t': 's3_barcode_status_t',
        'bc_decoder_t': 's3_barcode_decoder_t',
        'bc_decoder_record_state_t': 's3_rec_state_t',
        'ir_acquisition_config_valid': 's3_acquisition_config_valid',
        'ir_acquisition_capture': 's3_acquisition_capture',
        'ir_acquisition_sensor_id_t': 's3_sensor_id_t',
        'ir_acquisition_config_t': 's3_acquisition_config_t',
        'ir_acquisition_stored_t': 's3_acquisition_stored_config_t',
        'ir_acquisition_expand': 's3_acquisition_expand',
        'ir_acquisition_sample_t': 's3_raw_sample_t',
        'ir_acquisition_frame_t': 's3_sensor_frame_t',
        'ir_reflectance_surface_t': 'ir_surface_t',
        'ir_reflectance_calibration_t': 'ir_calibration_t',
        'ir_reflectance_stored_cal_t': 'ir_stored_calibration_t',
        'line_controller_defaults': 's3_steering_defaults',
        'line_controller_config_valid': 's3_steering_config_valid',
        'line_controller_compute': 's3_steering_compute',
        'line_controller_config_t': 's3_steering_config_t',
        'line_controller_request_t': 's3_steering_request_t',
        'course_profile_navigation': 's3_course_navigation',
        'course_profile_options': 's3_course_options',
        'remote_control_get_u32': 'remote_get32',
        'remote_control_put_u32': 'remote_put32',
        'remote_control_is_newer': 'remote_newer',
        'avoid_obstacle_init': 's5_avoidance_init',
        'avoid_obstacle_range_ready': 's5_range_ready',
        'avoid_obstacle_begin': 's5_avoidance_begin',
        'avoid_obstacle_abort': 's5_avoidance_abort',
        'avoid_obstacle_aim_deg': 's5_aim_deg',
        'avoid_obstacle_aim_deg_bend': 's5_aim_deg_bend',
        'avoid_obstacle_step': 's5_avoidance_step',
        'avoid_obstacle_state_t': 's5_state_t',
        'avoid_obstacle_fault_t': 's5_fault_t',
        'avoid_obstacle_action_kind_t': 's5_action_kind_t',
        'avoid_obstacle_pass_t': 's5_pass_t',
        'avoid_obstacle_action_t': 's5_action_t',
        'avoid_obstacle_input_t': 's5_input_t',
        'avoid_obstacle_t': 's5_avoidance_t',
        'avoid_obstacle_relative_get': 's5_rel_get',
        'avoid_obstacle_relative_set': 's5_rel_set',
        'avoid_obstacle_aim_get': 's5_aim_get',
        'avoid_obstacle_aim_set': 's5_aim_set',
        's5_codec_observation_pack': 's5_observation_pack',
        's5_codec_observation_unpack': 's5_observation_unpack',
        's5_codec_observation_t': 's5_observation_t',
        'ultrasonic_sensor_pulse_mm': 'ultrasonic_pulse_mm',
        'ultrasonic_sensor_echo_mm': 'ultrasonic_echo_mm',
        'wifi_retry_policy_action_t': 'wifi_retry_action_t',
        'wifi_retry_policy_schedule': 'wifi_retry_schedule',
        'wifi_retry_policy_join_result': 'wifi_retry_join_result',
        'wifi_retry_policy_init': 'wifi_retry_init',
        'wifi_retry_policy_observe': 'wifi_retry_observe',
        's3_drive_reset_result_t': 's3_reset_result_t',
        's3_integration_kind_t': 's3_control_kind_t',
        's3_integration_request_t': 's3_control_request_t',
        'telemetry_codec_action_t': 'buddy1_barcode_action_t',
        'telemetry_codec_route_t': 'buddy1_obstacle_route_t',
        'telemetry_codec_state_t': 'buddy1_robot_state_t',
        'telemetry_codec_map_t': 'buddy1_barcode_map_t',
        'telemetry_codec_avoidance_t': 'buddy1_avoidance_t',
        'telemetry_codec_scan_t': 'buddy1_scan_t',
        'telemetry_codec_terrain_t': 'buddy1_terrain_t',
        'telemetry_codec_scan_sample_t': 'buddy1_scan_sample_t',
        'telemetry_codec_snapshot_t': 'buddy1_snapshot_t',
        'motion_manoeuvre_t': 'manoeuvre_t',
        'wheel_encoder_id_t': 'wheel_id_t',
        'wheel_encoder_count_t': 'wheel_count_t',
        'bc_decoder_work_t': 's3_barcode_work_t',
        'bc_decoder_record_t': 's3_barcode_record_t',
        's5_api_snapshot_result_t': 's5_snapshot_result_t',
        'telemetry_codec_payload_size': 'buddy1_payload_size',
        'telemetry_codec_payload_bits': 'buddy1_payload_bits',
        'telemetry_codec_topic_name': 'buddy1_topic_name',
        'telemetry_codec_payload_valid': 'buddy1_payload_valid',
        'telemetry_codec_map_unpack': 'buddy1_map_unpack',
        'remote_control_link': 's3_remote_link',
        'remote_control_receive': 's3_remote_receive',
        'remote_control_reply_take': 's3_remote_reply_take',
        'remote_control_status_take': 's3_remote_status_take',
        'remote_control_stop_sequence': 's3_remote_stop_sequence',
        'remote_control_physical_stop': 's3_remote_physical_stop',
        'buddy4_read': 'subsystem4_read',
        'buddy4_snapshot': 'subsystem4_snapshot',
        'buddy4_event': 'subsystem4_event',
        'buddy4_reset_run': 'subsystem4_reset_run',
        'buddy4_begin_run': 'subsystem4_begin_run',
        'buddy4_set_mag_calibration': 'subsystem4_set_mag_calibration',
        's5_api_read_snapshot': 'subsystem5_read_snapshot',
        's5_api_snapshot': 'subsystem5_snapshot',
        's5_api_request_scan': 'subsystem5_request_scan',
        's5_api_request_peek': 'subsystem5_request_peek',
        's5_api_cancel_scan': 'subsystem5_cancel_scan',
        's5_api_set_control_status': 'subsystem5_set_control_status',
        's5_api_servo_angle': 'subsystem5_servo_angle',
        's5_api_scan_samples': 'subsystem5_scan_samples',
        's5_api_controller_status': 'subsystem5_controller_status',
        'op_panel_start': 'operator_panel_start',
        'run_timing_increment': 's3_timing_increment',
        'monotonic_clock_start': 'app_clock_start',
        'snapshot_motion_read': 'motion_read_snapshot',
        's5_config_defaults': 's5_defaults',
        'motion_manoeuvre_name': 'manoeuvre_name',
        'motion_wheel_left_distance_m': 'wheel_distance_left_m',
        'motion_wheel_right_distance_m': 'wheel_distance_right_m',
        'motion_wheel_left_speed_mps': 'wheel_speed_left_mps',
        'motion_wheel_right_speed_mps': 'wheel_speed_right_mps',
        'motion_robot_speed_mps': 'robot_speed_mps',
        'b_allows_cross_straight': 'cross_straight',
        'b_has_external_hold': 'external_hold',
        'b_waits_for_stop': 'waiting_stop',
        'b_has_obstacle_snapshot': 'obstacle_snapshot_valid',
        'b_is_bypassing': 'obstacle_active',
        'b_is_faint': 'faint',
        'b_is_rising': 'rising',
        'b_has_crossed': 'crossed',
        'b_is_strong': 'strong',
        'b_does_bias_vary': 'varying',
        'b_has_candidate': 'good',
        'b_allows_noise': 'allow_noise',
        'b_is_ambiguous': 'ambiguous',
        'b_is_provisional': 'provisional',
        'b_is_strict_fit_broken': 'strict_broken',
        'b_is_early': 'early',
        'b_accepts_faint_quiet': 'quiet_tolerant',
        'b_blocks_retry': 'retry_blocked',
        'b_did_try_early_fit': 'early_tried',
        'b_has_early_candidate': 'early_candidate',
        'b_is_connected': 'connected',
        'b_has_published_status': 'published',
        'b_is_stop_held': 'physical_stop',
        'b_has_active_request': 'inflight',
        'gb_faults_seen': 'g_b_faults_seen',
        'gb_is_capture_pending': 'g_bcap_net_pending',
        'gb_running': 'g_b_running',
        'gh_adc': 'g_h_adc',
        'gp_sink': 'g_p_sink',
        'b_is_configured': 'configured',
        'b_is_advancing': 'advancing',
        'b_is_searching': 'searching',
        'b_has_touched_line': 'touched',
        'b_did_command_succeed': 'command_ok',
        'b_is_collecting': 'collecting',
        'hump_detection_reset': 'hump_reset',
        'hump_detection_update': 'hump_update',
        'hump_detection_update_right': 'hump_update_right_wheel',
        'b_is_left_motor_inverted': 'b_m1_inverted',
        'b_is_right_motor_inverted': 'b_m2_inverted',
        'g_b_is_capture_pending': 'g_bcap_net_pending',
        'b_has_last_sample': 'have_last',
        'b_is_pulse_invalid': 'pulse_bad',
        'b_has_input_level': 'seen',
        'b_is_raw_pressed': 'raw',
        'b_is_stable_pressed': 'stable',
        'b_is_press_armed': 'armed',
        'b_is_awaiting_reply': 'awaiting_reply',
        'b_has_prompted': 'prompted',
        'b_has_handled_stop': 'stop_handled',
        'b_has_warned_stop': 'stop_warned',
        'b_is_go_inhibited': 'inhibit',
        'b_has_warned_ignore': 'ignore_warned',
        'b_should_reset_to_cal': 'reset_to_cal',
        'op_logic_press_t': 'op_press_t',
        'op_logic_button_t': 'op_button_t',
        'op_logic_phase_t': 'op_phase_t',
        'op_logic_port_t': 'op_port_t',
        'op_logic_t': 'operator_t',
        'op_logic_button_init': 'op_button_init',
        'op_logic_button_update': 'op_button_update',
        'op_logic_button_held': 'op_button_held',
        'op_logic_init': 'operator_init',
        'op_logic_remote_stop': 'operator_remote_stop',
        'op_logic_step': 'operator_step',
        'darkness_code': 'dark4',
        'noise_estimate': 'noise_q4',
        'pitch_tenths_deg': 'pitch_x10',
        'roll_tenths_deg': 'roll_x10',
        'integration_codec_pack_cmd': 's3_control_pack',
        'integration_codec_unpack_cmd': 's3_control_unpack',
        'integration_codec_pack_status': 's3_status_pack',
        'integration_codec_unpack_status': 's3_status_unpack',
        'navigation_codec_pack': 's3_navigation_pack',
        'navigation_codec_unpack': 's3_navigation_unpack',
        'navigation_codec_barcode_pack': 's3_barcode_pack',
        'navigation_codec_barcode_unpack': 's3_barcode_unpack',
        'navigation_codec_raw_pack': 's3_raw_frame_pack',
        'navigation_codec_raw_unpack': 's3_raw_frame_unpack',
        'navigation_codec_packet_t': 's3_navigation_packet_t',
        'navigation_codec_barcode_t': 's3_barcode_packet_t',
        'navigation_codec_raw_frame_t': 's3_raw_frame_packet_t',
    }
    return [f'-D{current}={legacy}' for current, legacy in legacy_names.items()]


def obstacle_predicate_flags(revision):
    """Keep the shared obstacle fixture bound to the original members."""
    if revision != FIXED_BASELINE:
        return []
    members = {
        'b_is_stopped': 'stopped',
        'b_is_motion_done': 'motion_done',
        'b_is_healthy': 'healthy',
        'b_has_tape': 'tape',
        'b_has_junction': 'junction',
        'b_is_line_ready': 'line_ready',
        'b_has_issued': 'issued',
        'b_has_swapped': 'swapped',
        'b_has_backed': 'backed',
        'b_is_backing': 'backing',
        'b_has_aimed': 'aimed',
        'b_is_aiming': 'aiming',
        'b_is_on_bend': 'bend',
        'b_is_after_tape': 'tail',
        'b_has_extended': 'extended',
        'b_has_alt_route': 'alt_ok',
        'b_is_touching': 'touching',
        'b_is_confirmed': 'confirmed',
        'b_is_valid': 'valid',
        'b_is_range_valid': 'range_valid',
        'b_has_no_echo': 'no_echo',
        'b_is_scan_done': 'scan_done',
        'b_is_fine': 'fine',
        'b_is_done': 'done',
        'b_has_failed': 'failed',
        'b_is_single': 'single',
    }
    return [f'-D{current}={legacy}' for current, legacy in members.items()]


def motion_predicate_flags(revision):
    """Bind shared motion fixture observations to the original fields."""
    if revision != FIXED_BASELINE:
        return []
    members = {
        'b_was_lost': 'b_lost',
        'b_is_stopped': 'b_stopped',
        'b_is_busy': 'b_busy',
        'b_is_status_valid': 'b_valid',
        'b_is_calibration_valid': 'b_valid',
        'b_is_left_enc_inverted': 'b_enc_l_inverted',
        'b_is_right_enc_inverted': 'b_enc_r_inverted',
        'b_is_left_enc_ok': 'b_enc_l_ok',
        'b_is_right_enc_ok': 'b_enc_r_ok',
        'b_are_encoders_swapped': 'b_encoders_swapped',
        'b_was_aborted': 'b_aborted',
        'b_is_feedback_valid': 'valid',
        'b_is_braked': 'braked',
        'b_is_debug_valid': 'valid',
        'b_is_debug_busy': 'busy',
        'b_is_cal_done': 'b_cal_done',
    }
    return [f'-D{current}={legacy}' for current, legacy in members.items()]


def terrain_predicate_flags(revision):
    """Bind shared terrain fixture observations to the original fields."""
    if revision != FIXED_BASELINE:
        return []
    members = {
        'b_is_hump_tracking': 'active',
        'b_is_armed': 'armed',
        'b_has_completed_hump': 'completed',
        'b_is_peak_valid': 'highest_valid',
        'b_is_hump_active': 'hump_active',
        'b_is_hump_complete': 'hump_completed',
        'b_has_impact': 'impact',
        'b_is_mag_valid': 'mag_valid',
        'b_is_calibrated': 'ready',
        'b_did_skip_acceleration': 'skipped_acceleration',
        'b_is_stationary': 'stationary',
        'b_is_valid': 'valid',
        'b_is_wheel_delta_valid': 'wheel_delta_valid',
    }
    return [f'-D{current}={legacy}' for current, legacy in members.items()]


def barcode_work_flags(root):
    """Select the fixed baseline's former extrema-count field for host fixtures."""
    header = (Path(root)/'app_program/subsystem3/bc_decoder.h').read_text()
    header = re.sub(r'/\*.*?\*/|//[^\n]*', '', header, flags=re.DOTALL)
    work = re.search(r'\btypedef\s+struct\s*\{([^{}]*)\}\s*(?:s3_barcode_work_t|bc_decoder_work_t)\s*;',
                     header)
    if work is None:
        raise ValueError('Missing barcode work record')
    counts = {field.strip() for group in re.findall(r'\buint16_t\s+([^;]+);',
                                                   work[1])
              for field in group.split(',')}
    if 'n' in counts and 'extrema_count' not in counts:
        return ['-DTEST_EXTREMA_COUNT_MEMBER=n']
    if 'extrema_count' in counts and 'n' not in counts:
        return []
    raise ValueError('Unrecognized extrema count field in barcode work record')


def tilt_module(root):
    """Accept exactly one complete pair, rejecting stale partial pairs and links."""
    directory = Path(root)/'app_program/subsystem4'
    names = ('tilt_estimation', 'tilt')
    if directory.is_symlink() or directory.parent.is_symlink():
        raise ValueError('Linked tilt module directory')
    present = {}
    for name in names:
        paths = [directory/(name+suffix) for suffix in ('.c', '.h')]
        if any(path.is_symlink() for path in paths):
            raise ValueError('Linked tilt module input')
        if any(not path.resolve().is_relative_to(Path(root).resolve()) for path in paths):
            raise ValueError('Tilt module input escapes subject source')
        present[name] = tuple(path.is_file() for path in paths)
    complete = [name for name, pair in present.items() if pair == (True, True)]
    if len(complete) != 1 or any(any(pair) for name, pair in present.items() if name not in complete):
        raise ValueError('Expected exactly one complete tilt .c/.h pair: '+str(present))
    return complete[0]


def prepare_tilt(root, revision, destination, receipt):
    """Generate only the fixed baseline's missing header, outside measured source."""
    root, destination, receipt = Path(root).resolve(), Path(destination).resolve(), Path(receipt)
    report = dict(subject_root=str(root), subject_revision=revision, status='FAIL', errors=[])
    try:
        name = tilt_module(root)
        report['module'] = name
        expected = 'tilt_estimation' if revision == FIXED_BASELINE else 'tilt'
        if not revision or name != expected:
            raise ValueError('Tilt filename pair does not match the selected revision boundary')
        header = root/'app_program/subsystem4'/(name+'.h')
        report['header'] = str(header)
        report['header_bytes_sha256'] = hashlib.sha256(header.read_bytes()).hexdigest()
        report['source_bytes_sha256'] = hashlib.sha256(header.with_suffix('.c').read_bytes()).hexdigest()
        includes = []
        if revision == FIXED_BASELINE:
            if report['header_bytes_sha256'] != BASELINE_TILT_HEADER_SHA256:
                raise ValueError('Fixed-baseline tilt header bytes differ from the reviewed Git blob')
            if destination.is_relative_to(root):
                raise ValueError('Historical header output must remain outside subject source')
            header_name = header.as_posix()
            if any(character in header_name for character in ('"', '\n', '\r')):
                raise ValueError('Unsupported historical header path quoting')
            destination.mkdir(parents=True, exist_ok=False)
            forwarder = destination/'tilt.h'
            contents = ('/* Derived fixture compatibility for fixed baseline '+FIXED_BASELINE+'. */\n'
                        '#include "'+header_name+'"\n')
            forwarder.write_text(contents, encoding='utf-8', newline='\n')
            report['derived_header'] = dict(path=str(forwarder), contents=contents,
                bytes_sha256=hashlib.sha256(forwarder.read_bytes()).hexdigest(),
                dependency_header=str(header), dependency_bytes_sha256=report['header_bytes_sha256'])
            includes.append(destination)
        report.update(status='PASS', include_directories=[str(path) for path in includes],
                      selection='fixed-baseline-derived-header' if includes else 'candidate-real-header')
        return name, includes
    except (OSError, ValueError) as error:
        report['errors'].append(str(error))
        raise
    finally:
        receipt.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
