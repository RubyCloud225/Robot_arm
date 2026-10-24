
## Test run: 2026-10-24T22:05:08+02:00

Exit status: 1

    ============================= test session starts ==============================
    platform darwin -- Python 3.14.6, pytest-9.1.1, pluggy-1.7.0 -- /Users/catherinepemblington/Documents/GitHub/Robot_arm/.venv/bin/python
    cachedir: .pytest_cache
    rootdir: /Users/catherinepemblington/Documents/GitHub/Robot_arm
    configfile: pyproject.toml
    collecting ... collected 602 items
    
    tests/test_ball.py::test_ball_has_uniform_solid_sphere_inertia[demo] PASSED [  0%]
    tests/test_ball.py::test_ball_has_uniform_solid_sphere_inertia[composed] PASSED [  0%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[demo-x] PASSED [  0%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[demo-y] PASSED [  0%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[demo-z] PASSED [  0%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[composed-x] PASSED [  0%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[composed-y] PASSED [  1%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[composed-z] PASSED [  1%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[demo-x] PASSED [  1%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[demo-y] PASSED [  1%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[demo-z] PASSED [  1%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[composed-x] PASSED [  1%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[composed-y] PASSED [  2%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[composed-z] PASSED [  2%]
    tests/test_ball.py::test_ball_uses_specified_sliding_friction_in_table_contact[demo] PASSED [  2%]
    tests/test_ball.py::test_ball_uses_specified_sliding_friction_in_table_contact[composed] PASSED [  2%]
    tests/test_ball.py::test_ball_free_joint_has_six_unconstrained_velocity_dofs[demo] PASSED [  2%]
    tests/test_ball.py::test_ball_free_joint_has_six_unconstrained_velocity_dofs[composed] PASSED [  2%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[demo-rest] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[demo-horizontal] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[demo-upward] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[demo-downward] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[composed-rest] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[composed-horizontal] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[composed-upward] PASSED [  4%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[composed-downward] PASSED [  4%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[demo-x] PASSED [  4%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[demo-y] PASSED [  4%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[demo-z] PASSED [  4%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[composed-x] PASSED [  4%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[composed-y] PASSED [  5%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[composed-z] PASSED [  5%]
    tests/test_ball.py::test_table_side_collision_changes_free_ball_momentum[demo-left] PASSED [  5%]
    tests/test_ball.py::test_table_side_collision_changes_free_ball_momentum[demo-right] PASSED [  5%]
    tests/test_ball.py::test_table_side_collision_changes_free_ball_momentum[composed-left] PASSED [  5%]
    tests/test_ball.py::test_table_side_collision_changes_free_ball_momentum[composed-right] PASSED [  5%]
    tests/test_cli.py::test_robot_model_requires_robot_flag PASSED           [  6%]
    tests/test_cli.py::test_robot_mode_reports_missing_model_path PASSED     [  6%]
    tests/test_config.py::test_default_config_resolves_packaged_model_and_values PASSED [  6%]
    tests/test_config.py::test_load_config_rejects_invalid_timing[0-0.04-timestep must be greater than zero] PASSED [  6%]
    tests/test_config.py::test_load_config_rejects_invalid_timing[true-0.04-timestep must be a number] PASSED [  6%]
    tests/test_config.py::test_load_config_rejects_invalid_timing[0.002-0.031-integer multiple] PASSED [  6%]
    tests/test_config.py::test_load_config_resolves_relative_model_path PASSED [  7%]
    tests/test_config.py::test_load_config_rejects_absolute_model_path PASSED [  7%]
    tests/test_config.py::test_load_config_rejects_missing_model PASSED      [  7%]
    tests/test_config.py::test_apply_config_applies_timestep_and_scene_positions PASSED [  7%]
    tests/test_config.py::test_apply_config_checks_joint_and_actuator_limits[{"finger_joint" = 1.5}-{}-outside its range] PASSED [  7%]
    tests/test_config.py::test_apply_config_checks_joint_and_actuator_limits[{}-{"gripper" = 1.5}-outside its range] PASSED [  7%]
    tests/test_config.py::test_apply_config_checks_joint_and_actuator_limits[{"unknown_joint" = 0.5}-{}-joint does not exist] PASSED [  8%]
    tests/test_config.py::test_apply_config_checks_joint_and_actuator_limits[{}-{"unknown_actuator" = 0.5}-actuator does not exist] PASSED [  8%]
    tests/test_contact_filters.py::test_robot_collision_masks_allow_scene_contacts_and_visuals_are_disabled PASSED [  8%]
    tests/test_contact_filters.py::test_ball_contacts_every_hand_and_finger_collider[hand] PASSED [  8%]
    tests/test_contact_filters.py::test_ball_contacts_every_hand_and_finger_collider[left_finger] PASSED [  8%]
    tests/test_contact_filters.py::test_ball_contacts_every_hand_and_finger_collider[right_finger] PASSED [  8%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[floor-hand] PASSED [  9%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[floor-left_finger] PASSED [  9%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[floor-right_finger] PASSED [  9%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[table_top-hand] PASSED [  9%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[table_top-left_finger] PASSED [  9%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[table_top-right_finger] PASSED [  9%]
    tests/test_contact_filters.py::test_original_base_link_exclusion_is_preserved PASSED [ 10%]
    tests/test_floor.py::test_floor_is_fixed_at_world_zero_with_debug_material PASSED [ 10%]
    tests/test_floor.py::test_floor_contact_boundary[overlap] PASSED         [ 10%]
    tests/test_floor.py::test_floor_contact_boundary[clearance] PASSED       [ 10%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[0.8-0] PASSED [ 10%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[-0.8-0] PASSED [ 10%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[0-0.8] PASSED [ 11%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[0--0.8] PASSED [ 11%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[2.5-0] PASSED [ 11%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[-2.5-0] PASSED [ 11%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[0-2.5] PASSED [ 11%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[0--2.5] PASSED [ 11%]
    tests/test_model_addresses.py::test_correct_joint_address_dimensions PASSED [ 12%]
    tests/test_model_addresses.py::test_qpos_and_qvel_addresses_are_not_confused PASSED [ 12%]
    tests/test_model_addresses.py::test_all_joint_slices_within_bounds PASSED [ 12%]
    tests/test_model_addresses.py::test_free_joint_slices_read_correct_dimensions PASSED [ 12%]
    tests/test_model_addresses.py::test_different_model_is_rejected PASSED   [ 12%]
    tests/test_model_addresses.py::test_wrong_joint_identifier_is_rejected PASSED [ 12%]
    tests/test_model_addresses.py::test_cross_wired_joint_identifier_is_rejected PASSED [ 13%]
    tests/test_model_addresses.py::test_negative_identifier_is_rejected PASSED [ 13%]
    tests/test_model_addresses.py::test_boolean_identifier_is_rejected PASSED [ 13%]
    tests/test_model_addresses.py::test_wrong_actuator_identifier_is_rejected PASSED [ 13%]
    tests/test_model_addresses.py::test_cache_rejects_mutation PASSED        [ 13%]
    tests/test_model_addresses.py::test_input_mapping_mutation_does_not_change_cache PASSED [ 13%]
    tests/test_model_addresses.py::test_missing_cached_joint_rejected PASSED [ 14%]
    tests/test_model_addresses.py::test_cache_build_does_not_mutate_model_or_state PASSED [ 14%]
    tests/test_model_addresses.py::test_correct_actuator_control_index PASSED [ 14%]
    tests/test_model_addresses.py::test_all_four_joint_types_have_correct_widths PASSED [ 14%]
    tests/test_model_addresses.py::test_all_joint_slices_cover_exact_state_dimensions PASSED [ 14%]
    tests/test_model_addresses.py::test_free_joint_is_not_treated_as_six_qpos_values PASSED [ 14%]
    tests/test_model_addresses.py::test_recompiled_equivalent_model_rejects_old_cache PASSED [ 15%]
    tests/test_model_addresses.py::test_same_numeric_joint_id_wrong_name_is_rejected PASSED [ 15%]
    tests/test_model_addresses.py::test_wrong_body_id_is_rejected PASSED     [ 15%]
    tests/test_model_addresses.py::test_wrong_site_id_is_rejected PASSED     [ 15%]
    tests/test_model_addresses.py::test_negative_body_id_rejected PASSED     [ 15%]
    tests/test_model_addresses.py::test_cached_mapping_is_read_only PASSED   [ 15%]
    tests/test_model_addresses.py::test_missing_actuator_lookup_fails PASSED [ 16%]
    tests/test_model_addresses.py::test_independent_caches_do_not_share_mapping_storage PASSED [ 16%]
    tests/test_model_addresses.py::test_panda_joint_and_actuator_address_cache PASSED [ 16%]
    tests/test_model_addresses.py::test_panda_cache_isolation_across_instances PASSED [ 16%]
    tests/test_model_diagnostics.py::test_real_panda_diagnostics_pass PASSED [ 16%]
    tests/test_model_diagnostics.py::test_diagnostic_metadata_matches_compiled_model PASSED [ 16%]
    tests/test_model_diagnostics.py::test_panda_expected_dimensions PASSED   [ 17%]
    tests/test_model_diagnostics.py::test_report_contains_all_required_stages PASSED [ 17%]
    tests/test_model_diagnostics.py::test_source_path_is_identification_only PASSED [ 17%]
    tests/test_model_diagnostics.py::test_blank_source_path_rejected[] PASSED [ 17%]
    tests/test_model_diagnostics.py::test_blank_source_path_rejected[   ] PASSED [ 17%]
    tests/test_model_diagnostics.py::test_invalid_model_argument_rejected[None] PASSED [ 17%]
    tests/test_model_diagnostics.py::test_invalid_model_argument_rejected[5] PASSED [ 18%]
    tests/test_model_diagnostics.py::test_invalid_model_argument_rejected[model] PASSED [ 18%]
    tests/test_model_diagnostics.py::test_invalid_source_type_rejected PASSED [ 18%]
    tests/test_model_diagnostics.py::test_missing_required_name_blocks_address_cache PASSED [ 18%]
    tests/test_model_diagnostics.py::test_disabled_joint_limit_fails_overall PASSED [ 18%]
    tests/test_model_diagnostics.py::test_nonfinite_actuator_range_is_reported PASSED [ 18%]
    tests/test_model_diagnostics.py::test_invalid_timestep_is_reported PASSED [ 19%]
    tests/test_model_diagnostics.py::test_diagnostic_report_is_immutable PASSED [ 19%]
    tests/test_model_diagnostics.py::test_report_is_snapshot_not_live_model_view PASSED [ 19%]
    tests/test_model_diagnostics.py::test_repeated_diagnostics_are_deterministic PASSED [ 19%]
    tests/test_model_diagnostics.py::test_text_report_contains_required_counts PASSED [ 19%]
    tests/test_model_diagnostics.py::test_failure_text_contains_actionable_error PASSED [ 19%]
    tests/test_model_diagnostics.py::test_blocked_stage_does_not_appear_as_pass PASSED [ 20%]
    tests/test_model_diagnostics.py::test_no_checks_is_not_a_success PASSED  [ 20%]
    tests/test_model_diagnostics.py::test_invalid_report_type_rejected PASSED [ 20%]
    tests/test_model_diagnostics.py::test_diagnostics_do_not_mutate_model_or_data PASSED [ 20%]
    tests/test_model_diagnostics.py::test_failure_diagnostics_do_not_mutate_model PASSED [ 20%]
    tests/test_model_diagnostics.py::test_formatter_does_not_print PASSED    [ 20%]
    tests/test_model_diagnostics.py::test_diagnostics_never_call_mj_step PASSED [ 21%]
    tests/test_model_diagnostics.py::test_invalid_physics_timestep_never_passes[nan] PASSED [ 21%]
    tests/test_model_diagnostics.py::test_invalid_physics_timestep_never_passes[inf] PASSED [ 21%]
    tests/test_model_diagnostics.py::test_invalid_physics_timestep_never_passes[-inf] PASSED [ 21%]
    tests/test_model_diagnostics.py::test_invalid_physics_timestep_never_passes[0.0] PASSED [ 21%]
    tests/test_model_diagnostics.py::test_invalid_physics_timestep_never_passes[-0.002] PASSED [ 21%]
    tests/test_model_diagnostics.py::test_malformed_model_reports_multiple_failures PASSED [ 22%]
    tests/test_model_diagnostics.py::test_unexpected_programming_error_is_not_masked PASSED [ 22%]
    tests/test_model_diagnostics.py::test_report_is_not_revalidated_when_formatted PASSED [ 22%]
    tests/test_model_ids.py::test_resolves_existing_names[object_type0-test_joint] PASSED [ 22%]
    tests/test_model_ids.py::test_resolves_existing_names[object_type1-test_actuator] PASSED [ 22%]
    tests/test_model_ids.py::test_resolves_existing_names[object_type2-test_body] PASSED [ 22%]
    tests/test_model_ids.py::test_resolves_existing_names[object_type3-test_site] PASSED [ 23%]
    tests/test_model_ids.py::test_missing_name_is_rejected[object_type0] PASSED [ 23%]
    tests/test_model_ids.py::test_missing_name_is_rejected[object_type1] PASSED [ 23%]
    tests/test_model_ids.py::test_missing_name_is_rejected[object_type2] PASSED [ 23%]
    tests/test_model_ids.py::test_missing_name_is_rejected[object_type3] PASSED [ 23%]
    tests/test_model_ids.py::test_wrong_object_category_rejected PASSED      [ 23%]
    tests/test_model_ids.py::test_blank_names_rejected[] PASSED              [ 24%]
    tests/test_model_ids.py::test_blank_names_rejected[ ] PASSED             [ 24%]
    tests/test_model_ids.py::test_blank_names_rejected[\t] PASSED            [ 24%]
    tests/test_model_ids.py::test_invalid_model_type_rejected PASSED         [ 24%]
    tests/test_model_ids.py::test_invalid_object_type_rejected PASSED        [ 24%]
    tests/test_model_ids.py::test_unsupported_object_category_rejected PASSED [ 24%]
    tests/test_model_ids.py::test_complete_custom_schema PASSED              [ 25%]
    tests/test_model_ids.py::test_missing_components_reported_together PASSED [ 25%]
    tests/test_model_ids.py::test_duplicate_names_rejected PASSED            [ 25%]
    tests/test_model_ids.py::test_no_physics_step_occurs PASSED              [ 25%]
    tests/test_model_ids.py::test_default_panda_schema_rejects_incomplete_model PASSED [ 25%]
    tests/test_model_ids.py::test_same_name_in_different_object_categories PASSED [ 25%]
    tests/test_model_ids.py::test_name_matching_is_exact[Test_Joint] PASSED  [ 26%]
    tests/test_model_ids.py::test_name_matching_is_exact[TEST_JOINT] PASSED  [ 26%]
    tests/test_model_ids.py::test_name_matching_is_exact[test_Joint] PASSED  [ 26%]
    tests/test_model_ids.py::test_name_matching_is_exact[test_joint ] PASSED [ 26%]
    tests/test_model_ids.py::test_name_matching_is_exact[ test_joint] PASSED [ 26%]
    tests/test_model_ids.py::test_name_matching_is_exact[test_joint\n] PASSED [ 26%]
    tests/test_model_ids.py::test_duplicate_names_across_joint_groups_rejected PASSED [ 27%]
    tests/test_model_ids.py::test_duplicate_names_across_actuator_groups_rejected PASSED [ 27%]
    tests/test_model_ids.py::test_duplicate_names_across_body_groups_rejected PASSED [ 27%]
    tests/test_model_ids.py::test_empty_schema_is_rejected_for_full_validation PASSED [ 27%]
    tests/test_model_ids.py::test_model_ids_recomputed_when_joint_order_changes PASSED [ 27%]
    tests/test_model_ids.py::test_validation_does_not_modify_compiled_model PASSED [ 27%]
    tests/test_model_ids.py::test_failed_validation_does_not_mutate_model PASSED [ 28%]
    tests/test_model_ids.py::test_resolved_result_does_not_depend_on_later_lookups PASSED [ 28%]
    tests/test_model_ids.py::test_missing_site_does_not_fallback_to_body PASSED [ 28%]
    tests/test_model_ids.py::test_unexpected_schema_type_is_rejected PASSED  [ 28%]
    tests/test_model_instance.py::test_panda_multi_instance_state_isolation PASSED [ 28%]
    tests/test_model_instance.py::test_instance_contains_correct_types PASSED [ 28%]
    tests/test_model_instance.py::test_separate_instances_have_distinct_models PASSED [ 29%]
    tests/test_model_instance.py::test_initial_states_are_equivalent PASSED  [ 29%]
    tests/test_model_instance.py::test_joint_positions_are_independent PASSED [ 29%]
    tests/test_model_instance.py::test_joint_velocities_are_independent PASSED [ 29%]
    tests/test_model_instance.py::test_actuator_commands_are_independent PASSED [ 29%]
    tests/test_model_instance.py::test_mutable_model_properties_are_independent PASSED [ 29%]
    tests/test_model_instance.py::test_stepping_one_instance_does_not_step_another PASSED [ 30%]
    tests/test_model_instance.py::test_repeated_stepping_remains_isolated PASSED [ 30%]
    tests/test_model_instance.py::test_attribute_reassignment_is_rejected PASSED [ 30%]
    tests/test_model_instance.py::test_instance_keeps_model_and_data_alive PASSED [ 30%]
    tests/test_model_instance.py::test_multiple_instances_remain_usable PASSED [ 30%]
    tests/test_model_instance.py::test_loading_has_no_physics_time_advance PASSED [ 30%]
    tests/test_model_instance.py::test_source_model_remains_unchanged PASSED [ 31%]
    tests/test_model_instance.py::test_invalid_file_propagates_loader_error PASSED [ 31%]
    tests/test_model_instance.py::test_real_panda_instances_are_independent PASSED [ 31%]
    tests/test_model_instance.py::test_public_constructor_rejected PASSED    [ 31%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[model] PASSED [ 31%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[data] PASSED [ 31%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[_model] PASSED [ 32%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[_data] PASSED [ 32%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[_sealed] PASSED [ 32%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[extra] PASSED [ 32%]
    tests/test_model_instance.py::test_ownership_attribute_deletion_rejected[model] PASSED [ 32%]
    tests/test_model_instance.py::test_ownership_attribute_deletion_rejected[data] PASSED [ 32%]
    tests/test_model_instance.py::test_ownership_attribute_deletion_rejected[_model] PASSED [ 33%]
    tests/test_model_instance.py::test_ownership_attribute_deletion_rejected[_data] PASSED [ 33%]
    tests/test_model_instance.py::test_ownership_attribute_deletion_rejected[_sealed] PASSED [ 33%]
    tests/test_model_instance.py::test_shallow_copy_is_rejected PASSED       [ 33%]
    tests/test_model_instance.py::test_deep_copy_is_rejected PASSED          [ 33%]
    tests/test_model_instance.py::test_pickle_roundtrip_is_rejected PASSED   [ 33%]
    tests/test_model_instance.py::test_independent_arrays_do_not_share_memory PASSED [ 34%]
    tests/test_model_instance.py::test_state_mutation_does_not_cross_instances PASSED [ 34%]
    tests/test_model_instance.py::test_reverse_direction_isolation PASSED    [ 34%]
    tests/test_model_instance.py::test_mutable_model_arrays_are_independent PASSED [ 34%]
    tests/test_model_instance.py::test_repeated_creation_does_not_reuse_models PASSED [ 34%]
    tests/test_model_instance.py::test_gc_does_not_corrupt_surviving_instance PASSED [ 34%]
    tests/test_model_instance.py::test_failed_load_produces_no_instance PASSED [ 35%]
    tests/test_model_instance.py::test_failed_scene_creation_produces_no_instance PASSED [ 35%]
    tests/test_model_instance.py::test_invalid_internal_pair_types_rejected PASSED [ 35%]
    tests/test_model_instance.py::test_source_file_hash_remains_unchanged PASSED [ 35%]
    tests/test_model_instance.py::test_parallel_interleaved_simulations_isolated PASSED [ 35%]
    tests/test_model_instance.py::test_subclass_factory_is_rejected PASSED   [ 35%]
    tests/test_model_limits.py::test_real_panda_limits_valid PASSED          [ 36%]
    tests/test_model_limits.py::test_all_home_values_inside_bounds PASSED    [ 36%]
    tests/test_model_limits.py::test_invalid_joint_defaults_rejected[nan] PASSED [ 36%]
    tests/test_model_limits.py::test_invalid_joint_defaults_rejected[inf] PASSED [ 36%]
    tests/test_model_limits.py::test_invalid_joint_defaults_rejected[-inf] PASSED [ 36%]
    tests/test_model_limits.py::test_invalid_joint_defaults_rejected[True] PASSED [ 36%]
    tests/test_model_limits.py::test_invalid_joint_defaults_rejected[invalid] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[nan] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[inf] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[-inf] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[-1.0] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[256.0] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[True] PASSED [ 38%]
    tests/test_model_limits.py::test_joint_limit_disabled_rejected PASSED    [ 38%]
    tests/test_model_limits.py::test_actuator_limit_disabled_rejected PASSED [ 38%]
    tests/test_model_limits.py::test_nonfinite_joint_bound_rejected[nan] PASSED [ 38%]
    tests/test_model_limits.py::test_nonfinite_joint_bound_rejected[inf] PASSED [ 38%]
    tests/test_model_limits.py::test_nonfinite_joint_bound_rejected[-inf] PASSED [ 38%]
    tests/test_model_limits.py::test_reversed_actuator_range_rejected PASSED [ 39%]
    tests/test_model_limits.py::test_corrupted_home_position_rejected PASSED [ 39%]
    tests/test_model_limits.py::test_gripper_range_is_not_metres PASSED      [ 39%]
    tests/test_model_limits.py::test_validator_preserves_model PASSED        [ 39%]
    tests/test_model_loader.py::test_valid_model_loads PASSED                [ 39%]
    tests/test_model_loader.py::test_string_path_is_supported PASSED         [ 39%]
    tests/test_model_loader.py::test_relative_path_is_supported PASSED       [ 40%]
    tests/test_model_loader.py::test_missing_file_has_useful_error PASSED    [ 40%]
    tests/test_model_loader.py::test_directory_is_rejected PASSED            [ 40%]
    tests/test_model_loader.py::test_empty_paths_are_rejected[] PASSED       [ 40%]
    tests/test_model_loader.py::test_empty_paths_are_rejected[ ] PASSED      [ 40%]
    tests/test_model_loader.py::test_empty_paths_are_rejected[\t] PASSED     [ 40%]
    tests/test_model_loader.py::test_empty_paths_are_rejected[\n] PASSED     [ 41%]
    tests/test_model_loader.py::test_invalid_path_types_are_rejected[None] PASSED [ 41%]
    tests/test_model_loader.py::test_invalid_path_types_are_rejected[123] PASSED [ 41%]
    tests/test_model_loader.py::test_invalid_path_types_are_rejected[1.5] PASSED [ 41%]
    tests/test_model_loader.py::test_invalid_path_types_are_rejected[path3] PASSED [ 41%]
    tests/test_model_loader.py::test_invalid_path_types_are_rejected[path4] PASSED [ 41%]
    tests/test_model_loader.py::test_malformed_xml_is_rejected PASSED        [ 42%]
    tests/test_model_loader.py::test_empty_file_is_rejected PASSED           [ 42%]
    tests/test_model_loader.py::test_invalid_mujoco_element_is_rejected PASSED [ 42%]
    tests/test_model_loader.py::test_unknown_joint_reference_is_rejected PASSED [ 42%]
    tests/test_model_loader.py::test_missing_mesh_dependency_is_rejected PASSED [ 42%]
    tests/test_model_loader.py::test_model_loading_preserves_original_error PASSED [ 42%]
    tests/test_model_loader.py::test_model_file_is_not_modified PASSED       [ 43%]
    tests/test_model_loader.py::test_loading_produces_independent_states PASSED [ 43%]
    tests/test_model_loader.py::test_loading_does_not_step_physics PASSED    [ 43%]
    tests/test_model_loader.py::test_invalid_content_fails_cleanly[<not_mujoco/>] PASSED [ 43%]
    tests/test_model_loader.py::test_invalid_content_fails_cleanly[<mujoco><worldbody>] PASSED [ 43%]
    tests/test_model_loader.py::test_invalid_content_fails_cleanly[<mujoco><worldbody/><extra/></mujoco>] PASSED [ 43%]
    tests/test_model_loader.py::test_invalid_content_fails_cleanly[\x00] PASSED [ 44%]
    tests/test_model_structure.py::test_valid_structure_passes PASSED        [ 44%]
    tests/test_model_structure.py::test_correct_total_state_dimensions PASSED [ 44%]
    tests/test_model_structure.py::test_missing_arm_joint_rejected PASSED    [ 44%]
    tests/test_model_structure.py::test_extra_joint_rejected PASSED          [ 44%]
    tests/test_model_structure.py::test_wrong_finger_joint_type_rejected[hinge] PASSED [ 44%]
    tests/test_model_structure.py::test_wrong_finger_joint_type_rejected[ball] PASSED [ 45%]
    tests/test_model_structure.py::test_ball_must_be_free_joint[hinge] PASSED [ 45%]
    tests/test_model_structure.py::test_ball_must_be_free_joint[slide] PASSED [ 45%]
    tests/test_model_structure.py::test_ball_must_be_free_joint[ball] PASSED [ 45%]
    tests/test_model_structure.py::test_free_joint_attached_to_wrong_body_rejected PASSED [ 45%]
    tests/test_model_structure.py::test_missing_actuator_rejected[actuator1] PASSED [ 45%]
    tests/test_model_structure.py::test_missing_actuator_rejected[actuator4] PASSED [ 46%]
    tests/test_model_structure.py::test_missing_actuator_rejected[actuator7] PASSED [ 46%]
    tests/test_model_structure.py::test_missing_actuator_rejected[actuator8] PASSED [ 46%]
    tests/test_model_structure.py::test_extra_actuator_rejected PASSED       [ 46%]
    tests/test_model_structure.py::test_wrong_model_argument_rejected PASSED [ 46%]
    tests/test_model_structure.py::test_validation_is_read_only PASSED       [ 46%]
    tests/test_model_structure.py::test_failure_does_not_modify_state PASSED [ 47%]
    tests/test_model_structure.py::test_real_panda_structure PASSED          [ 47%]
    tests/test_model_structure.py::test_challenge_extra_joint_with_valid_required_names PASSED [ 47%]
    tests/test_model_structure.py::test_challenge_extra_joint_failure_is_read_only PASSED [ 47%]
    tests/test_model_structure.py::test_challenge_ball_free_joint_on_wrong_body PASSED [ 47%]
    tests/test_model_structure.py::test_challenge_wrong_ball_joint_does_not_advance_physics PASSED [ 47%]
    tests/test_model_structure.py::test_challenge_arm_joint_type_changed_with_same_dimensions PASSED [ 48%]
    tests/test_robot_placement.py::test_base_and_pedestal_stay_rigid PASSED  [ 48%]
    tests/test_robot_placement.py::test_movable_base_is_rejected[<freejoint/>] PASSED [ 48%]
    tests/test_robot_placement.py::test_movable_base_is_rejected[<joint type="hinge"/>] PASSED [ 48%]
    tests/test_robot_placement.py::test_fixed_geometry_penetration_is_rejected[0.65 0 -0.025-table_top] PASSED [ 48%]
    tests/test_robot_placement.py::test_fixed_geometry_penetration_is_rejected[0.26 -0.29 -0.275-table_leg_front_left] PASSED [ 48%]
    tests/test_robot_placement.py::test_fixed_geometry_penetration_is_rejected[1.5 0 -0.5-floor] PASSED [ 49%]
    tests/test_robot_placement.py::test_fixed_geometry_penetration_is_rejected[0 0 -0.1-robot_pedestal] PASSED [ 49%]
    tests/test_robot_placement.py::test_visual_only_geometry_does_not_block_reset PASSED [ 49%]
    tests/test_robot_placement.py::test_changed_reset_pose_is_checked PASSED [ 49%]
    tests/test_robot_placement.py::test_panda_home_and_nearby_sweeps_clear_supports PASSED [ 49%]
    tests/test_robot_placement.py::test_panda_base_stays_fixed_and_home_remains_clear PASSED [ 49%]
    tests/test_scene_settings.py::test_explicit_physics_options_survive_configuration[demo] PASSED [ 50%]
    tests/test_scene_settings.py::test_explicit_physics_options_survive_configuration[composed] PASSED [ 50%]
    tests/test_scene_settings.py::test_fixed_cameras_frame_table_ball_goal_and_robot_envelope[demo-overview] PASSED [ 50%]
    tests/test_scene_settings.py::test_fixed_cameras_frame_table_ball_goal_and_robot_envelope[demo-top_down] PASSED [ 50%]
    tests/test_scene_settings.py::test_fixed_cameras_frame_table_ball_goal_and_robot_envelope[composed-overview] PASSED [ 50%]
    tests/test_scene_settings.py::test_fixed_cameras_frame_table_ball_goal_and_robot_envelope[composed-top_down] PASSED [ 50%]
    tests/test_scene_settings.py::test_debug_lights_and_colours_are_distinct[demo] PASSED [ 50%]
    tests/test_scene_settings.py::test_debug_lights_and_colours_are_distinct[composed] PASSED [ 51%]
    tests/test_scene_units.py::test_gravity_is_earth_like_and_points_down PASSED [ 51%]
    tests/test_scene_units.py::test_scene_uses_radians_and_si_scale PASSED   [ 51%]
    tests/test_scene_units.py::test_robot_scene_keeps_gravity PASSED         [ 51%]
    tests/test_scene_units.py::test_free_fall_acceleration_matches_gravity PASSED [ 51%]
    tests/test_simulation.py::test_build_scene_attaches_robot_model PASSED   [ 51%]
    tests/test_simulation.py::test_build_scene_rejects_missing_model PASSED  [ 52%]
    tests/test_simulation.py::test_build_scene_rejects_invalid_xml PASSED    [ 52%]
    tests/test_simulation.py::test_build_scene_requires_home_keyframe PASSED [ 52%]
    tests/test_simulation.py::test_build_scene_copies_articulated_joint_types_from_home_keyframe PASSED [ 52%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.002-0.002-1] PASSED [ 52%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.01-0.002-5] PASSED [ 52%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.04-0.002-20] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.08-0.002-40] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.03-0.003-10] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.001-0.001-1] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_noninteger_or_short_periods_are_rejected[0.031-0.002] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_noninteger_or_short_periods_are_rejected[0.025-0.004] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_noninteger_or_short_periods_are_rejected[0.001-0.002] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_noninteger_or_short_periods_are_rejected[0.0001-0.002] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_noninteger_or_short_periods_are_rejected[0.0401-0.002] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_invalid_period_values_are_rejected[0.0] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_invalid_period_values_are_rejected[-0.04] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_invalid_period_values_are_rejected[nan] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_invalid_period_values_are_rejected[inf] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_values_are_rejected[-inf] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[True] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[False] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[None] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[0.04] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[period4] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[period5] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_invalid_physics_timestep_is_rejected[0.0] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_invalid_physics_timestep_is_rejected[-0.002] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_invalid_physics_timestep_is_rejected[nan] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_invalid_physics_timestep_is_rejected[inf] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_float_representation_noise_is_accepted PASSED [ 57%]
    tests/test_simulator_controller_period.py::test_large_finite_but_nonrepresentable_ratio_rejected PASSED [ 57%]
    tests/test_simulator_controller_period.py::test_default_controller_period_advances_20_steps FAILED [ 57%]
    tests/test_simulator_controller_period.py::test_one_controller_period_calls_mujoco_20_times FAILED [ 57%]
    tests/test_simulator_controller_period.py::test_controller_period_holds_command_constant FAILED [ 57%]
    tests/test_simulator_controller_period.py::test_none_control_retains_existing_commands FAILED [ 57%]
    tests/test_simulator_controller_period.py::test_controller_period_uses_current_model_timestep FAILED [ 58%]
    tests/test_simulator_controller_period.py::test_invalid_period_preserves_state_and_control FAILED [ 58%]
    tests/test_simulator_controller_period.py::test_invalid_period_does_not_call_physics FAILED [ 58%]
    tests/test_simulator_controller_period.py::test_invalid_control_does_not_advance_controller_period FAILED [ 58%]
    tests/test_simulator_controller_period.py::test_missing_period_is_rejected FAILED [ 58%]
    tests/test_simulator_controller_period.py::test_repeated_controller_period_calls_accumulate_time FAILED [ 58%]
    tests/test_simulator_controller_period.py::test_physics_step_and_controller_step_can_be_mixed FAILED [ 59%]
    tests/test_simulator_controller_period.py::test_reset_after_controller_period_restores_baseline FAILED [ 59%]
    tests/test_simulator_controller_period.py::test_controller_period_preserves_model_data_identity FAILED [ 59%]
    tests/test_simulator_controller_period.py::test_two_simulators_have_independent_controller_time FAILED [ 59%]
    tests/test_simulator_controller_period.py::test_controller_period_before_load_is_rejected FAILED [ 59%]
    tests/test_simulator_controller_period.py::test_controller_period_after_close_is_rejected FAILED [ 59%]
    tests/test_simulator_forward.py::test_reset_refreshes_all_body_positions PASSED [ 60%]
    tests/test_simulator_forward.py::test_reset_refreshes_body_orientations PASSED [ 60%]
    tests/test_simulator_forward.py::test_reset_refreshes_all_site_positions PASSED [ 60%]
    tests/test_simulator_forward.py::test_reset_refreshes_collision_geometry_positions PASSED [ 60%]
    tests/test_simulator_forward.py::test_ball_world_position_matches_restored_freejoint PASSED [ 60%]
    tests/test_simulator_forward.py::test_target_marker_position_is_refreshed PASSED [ 60%]
    tests/test_simulator_forward.py::test_reset_recomputes_current_contacts PASSED [ 61%]
    tests/test_simulator_forward.py::test_reset_preserves_primary_state_after_forward PASSED [ 61%]
    tests/test_simulator_forward.py::test_reset_does_not_advance_simulation_time PASSED [ 61%]
    tests/test_simulator_forward.py::test_reset_does_not_call_mj_step PASSED [ 61%]
    tests/test_simulator_forward.py::test_reset_calls_forward_exactly_once PASSED [ 61%]
    tests/test_simulator_forward.py::test_forward_occurs_after_state_restoration PASSED [ 61%]
    tests/test_simulator_forward.py::test_applied_forces_remain_cleared_after_forward PASSED [ 62%]
    tests/test_simulator_forward.py::test_reset_keeps_model_and_data_objects PASSED [ 62%]
    tests/test_simulator_forward.py::test_repeated_resets_have_stable_derived_state PASSED [ 62%]
    tests/test_simulator_forward.py::test_rejected_seed_does_not_call_forward PASSED [ 62%]
    tests/test_simulator_forward.py::test_reset_before_loading_is_rejected PASSED [ 62%]
    tests/test_simulator_forward.py::test_reset_after_closure_is_rejected PASSED [ 62%]
    tests/test_simulator_forward.py::test_independent_simulators_do_not_share_refresh_state PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_new_simulator_does_not_load_model PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_explicit_load_creates_model_and_data PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_independent_simulators_have_independent_models PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_loading_one_simulator_does_not_load_another PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_model_access_before_load_rejected PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_data_access_before_load_rejected PASSED [ 64%]
    tests/test_simulator_lifecycle.py::test_duplicate_load_rejected PASSED   [ 64%]
    tests/test_simulator_lifecycle.py::test_failed_load_preserves_new_state PASSED [ 64%]
    tests/test_simulator_lifecycle.py::test_close_transitions_to_closed PASSED [ 64%]
    tests/test_simulator_lifecycle.py::test_close_before_load_is_safe PASSED [ 64%]
    tests/test_simulator_lifecycle.py::test_double_close_is_safe PASSED      [ 64%]
    tests/test_simulator_lifecycle.py::test_access_after_close_rejected PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_reload_after_close_rejected PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_reset_before_load_rejected PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_step_before_load_rejected PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_step_is_available_after_load PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_invalid_constructor_path_rejected[None] PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_invalid_constructor_path_rejected[17] PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_invalid_constructor_path_rejected[2.5] PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_invalid_constructor_path_rejected[True] PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_invalid_constructor_path_rejected[] PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_load_does_not_advance_physics PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_close_does_not_advance_physics PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_close_does_not_destroy_external_reference PASSED [ 67%]
    tests/test_simulator_lifecycle.py::test_instances_remain_independent_after_one_closes PASSED [ 67%]
    tests/test_simulator_reset.py::test_reset_restores_entire_primary_state PASSED [ 67%]
    tests/test_simulator_reset.py::test_reset_restores_all_seven_arm_joints PASSED [ 67%]
    tests/test_simulator_reset.py::test_reset_restores_both_finger_joints PASSED [ 67%]
    tests/test_simulator_reset.py::test_reset_restores_ball_translation_and_quaternion PASSED [ 67%]
    tests/test_simulator_reset.py::test_reset_restores_all_ball_velocities PASSED [ 68%]
    tests/test_simulator_reset.py::test_reset_restores_every_actuator_command PASSED [ 68%]
    tests/test_simulator_reset.py::test_reset_restores_target_position PASSED [ 68%]
    tests/test_simulator_reset.py::test_reset_restores_time_without_stepping PASSED [ 68%]
    tests/test_simulator_reset.py::test_reset_preserves_model_and_data_identity PASSED [ 68%]
    tests/test_simulator_reset.py::test_repeated_reset_is_idempotent PASSED  [ 68%]
    tests/test_simulator_reset.py::test_reset_snapshot_is_not_a_live_view PASSED [ 69%]
    tests/test_simulator_reset.py::test_reset_does_not_replace_other_instance_state PASSED [ 69%]
    tests/test_simulator_reset.py::test_reset_before_load_is_rejected PASSED [ 69%]
    tests/test_simulator_reset.py::test_reset_after_close_is_rejected PASSED [ 69%]
    tests/test_simulator_reset.py::test_seeded_reset_not_silently_accepted PASSED [ 69%]
    tests/test_simulator_reset.py::test_unsupported_seed_does_not_change_state PASSED [ 69%]
    tests/test_simulator_reset.py::test_reset_does_not_advance_physics PASSED [ 70%]
    tests/test_simulator_reset.py::test_reset_does_not_recompile_scene PASSED [ 70%]
    tests/test_simulator_reset.py::test_initial_snapshot_is_cleared_on_close PASSED [ 70%]
    tests/test_simulator_reset.py::test_snapshot_arrays_are_read_only PASSED [ 70%]
    tests/test_simulator_reset.py::test_step_then_reset_restores_original_state PASSED [ 70%]
    tests/test_simulator_stepping.py::test_one_step_advances_one_physics_timestep PASSED [ 70%]
    tests/test_simulator_stepping.py::test_time_advances_by_exact_substep_count[1] PASSED [ 71%]
    tests/test_simulator_stepping.py::test_time_advances_by_exact_substep_count[2] PASSED [ 71%]
    tests/test_simulator_stepping.py::test_time_advances_by_exact_substep_count[5] PASSED [ 71%]
    tests/test_simulator_stepping.py::test_time_advances_by_exact_substep_count[20] PASSED [ 71%]
    tests/test_simulator_stepping.py::test_time_advances_by_exact_substep_count[40] PASSED [ 71%]
    tests/test_simulator_stepping.py::test_stepping_twice_accumulates_physics_time PASSED [ 71%]
    tests/test_simulator_stepping.py::test_step_uses_loaded_model_timestep PASSED [ 72%]
    tests/test_simulator_stepping.py::test_step_advances_ball_under_gravity PASSED [ 72%]
    tests/test_simulator_stepping.py::test_no_control_preserves_existing_command PASSED [ 72%]
    tests/test_simulator_stepping.py::test_valid_control_vector_is_written PASSED [ 72%]
    tests/test_simulator_stepping.py::test_valid_gripper_control_is_not_treated_as_metres PASSED [ 72%]
    tests/test_simulator_stepping.py::test_numpy_array_and_list_controls_are_accepted PASSED [ 72%]
    tests/test_simulator_stepping.py::test_input_command_is_copied_not_modified PASSED [ 73%]
    tests/test_simulator_stepping.py::test_nonpositive_substeps_rejected_without_stepping[0] PASSED [ 73%]
    tests/test_simulator_stepping.py::test_nonpositive_substeps_rejected_without_stepping[-1] PASSED [ 73%]
    tests/test_simulator_stepping.py::test_nonpositive_substeps_rejected_without_stepping[-20] PASSED [ 73%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[True] PASSED [ 73%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[False] PASSED [ 73%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[1.0] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[2] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[None] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[count5] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_wrong_control_shape_rejected[invalid0] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_wrong_control_shape_rejected[invalid1] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_wrong_control_shape_rejected[invalid2] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_wrong_control_shape_rejected[invalid3] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_multidimensional_control_rejected[invalid0] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_multidimensional_control_rejected[invalid1] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_nonfinite_controls_rejected[invalid0] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_nonfinite_controls_rejected[invalid1] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_nonfinite_controls_rejected[invalid2] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_non_numeric_control_vectors_rejected[invalid0] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_non_numeric_control_vectors_rejected[invalid1] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_non_numeric_control_vectors_rejected[invalid2] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_invalid_control_representations_rejected[True] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_invalid_control_representations_rejected[1.5] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_invalid_control_representations_rejected[123] PASSED [ 77%]
    tests/test_simulator_stepping.py::test_invalid_control_representations_rejected[invalid3] PASSED [ 77%]
    tests/test_simulator_stepping.py::test_out_of_range_gripper_control_rejected PASSED [ 77%]
    tests/test_simulator_stepping.py::test_negative_gripper_control_rejected PASSED [ 77%]
    tests/test_simulator_stepping.py::test_invalid_control_does_not_partially_write PASSED [ 77%]
    tests/test_simulator_stepping.py::test_invalid_timestep_rejected[0.0] PASSED [ 77%]
    tests/test_simulator_stepping.py::test_invalid_timestep_rejected[-0.002] PASSED [ 78%]
    tests/test_simulator_stepping.py::test_invalid_timestep_rejected[nan] PASSED [ 78%]
    tests/test_simulator_stepping.py::test_invalid_timestep_rejected[inf] PASSED [ 78%]
    tests/test_simulator_stepping.py::test_model_data_identity_preserved_during_stepping PASSED [ 78%]
    tests/test_simulator_stepping.py::test_step_invokes_mujoco_exactly_n_times PASSED [ 78%]
    tests/test_simulator_stepping.py::test_invalid_control_never_invokes_mujoco_step PASSED [ 78%]
    tests/test_simulator_stepping.py::test_control_is_held_constant_across_substeps PASSED [ 79%]
    tests/test_simulator_stepping.py::test_reset_after_stepping_restores_initial_time_and_state PASSED [ 79%]
    tests/test_simulator_stepping.py::test_multiple_instances_advance_independently PASSED [ 79%]
    tests/test_simulator_stepping.py::test_step_before_load_is_rejected PASSED [ 79%]
    tests/test_simulator_stepping.py::test_step_after_close_is_rejected PASSED [ 79%]
    tests/test_simulator_stepping.py::test_step_does_not_create_viewer PASSED [ 79%]
    tests/test_simulator_stepping.py::test_none_control_does_not_attempt_array_conversion PASSED [ 80%]
    tests/test_simulator_transients.py::test_generalised_applied_forces_are_cleared PASSED [ 80%]
    tests/test_simulator_transients.py::test_external_body_forces_are_cleared PASSED [ 80%]
    tests/test_simulator_transients.py::test_solver_warmstart_is_cleared PASSED [ 80%]
    tests/test_simulator_transients.py::test_transients_are_cleared_together PASSED [ 80%]
    tests/test_simulator_transients.py::test_reset_restores_captured_actuator_activation PASSED [ 80%]
    tests/test_simulator_transients.py::test_reset_does_not_zero_valid_control_commands PASSED [ 81%]
    tests/test_simulator_transients.py::test_reset_restores_joint_state_and_clears_forces PASSED [ 81%]
    tests/test_simulator_transients.py::test_reset_preserves_ball_quaternion PASSED [ 81%]
    tests/test_simulator_transients.py::test_reset_restores_target_and_clears_forces PASSED [ 81%]
    tests/test_simulator_transients.py::test_reset_restores_time_and_clears_warmstart PASSED [ 81%]
    tests/test_simulator_transients.py::test_repeated_resets_do_not_reintroduce_forces PASSED [ 81%]
    tests/test_simulator_transients.py::test_two_simulators_have_independent_transients PASSED [ 82%]
    tests/test_simulator_transients.py::test_reset_keeps_model_and_data_identity PASSED [ 82%]
    tests/test_simulator_transients.py::test_reset_does_not_call_mj_step PASSED [ 82%]
    tests/test_simulator_transients.py::test_reset_refreshes_forward_state PASSED [ 82%]
    tests/test_simulator_transients.py::test_native_reset_is_called_once PASSED [ 82%]
    tests/test_simulator_transients.py::test_unsupported_seed_preserves_all_transients PASSED [ 82%]
    tests/test_simulator_transients.py::test_reset_after_close_is_rejected PASSED [ 83%]
    tests/test_simulator_transients.py::test_reset_does_not_modify_model_joint_limits PASSED [ 83%]
    tests/test_simulator_transients.py::test_no_viewer_is_required PASSED    [ 83%]
    tests/test_simulator_transients.py::test_corrupt_snapshot_rejected_before_native_reset PASSED [ 83%]
    tests/test_spawn_and_target.py::test_default_spawn_has_clearance_and_settles_without_explosive_correction[demo] PASSED [ 83%]
    tests/test_spawn_and_target.py::test_default_spawn_has_clearance_and_settles_without_explosive_correction[composed] PASSED [ 83%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[demo-0.54] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[demo-0.540001] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[demo-0.62] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[composed-0.54] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[composed-0.540001] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[composed-0.62] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position0-table_top] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position1-table_top] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position2-floor] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position3-table_leg_front_left] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position4-table_leg_front_right] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position5-table_leg_back_left] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position6-table_leg_back_right] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position0-table_top] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position1-table_top] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position2-floor] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position3-table_leg_front_left] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position4-table_leg_front_right] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position5-table_leg_back_left] PASSED [ 87%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position6-table_leg_back_right] PASSED [ 87%]
    tests/test_spawn_and_target.py::test_spawn_inside_robot_is_rejected PASSED [ 87%]
    tests/test_spawn_and_target.py::test_target_is_visible_fixed_site_without_collision_geometry[demo] PASSED [ 87%]
    tests/test_spawn_and_target.py::test_target_is_visible_fixed_site_without_collision_geometry[composed] PASSED [ 87%]
    tests/test_spawn_and_target.py::test_target_tracks_configured_position[demo] PASSED [ 87%]
    tests/test_spawn_and_target.py::test_target_tracks_configured_position[composed] PASSED [ 88%]
    tests/test_spawn_and_target.py::test_target_overlap_cannot_change_ball_trajectory[position0] PASSED [ 88%]
    tests/test_spawn_and_target.py::test_target_overlap_cannot_change_ball_trajectory[position1] PASSED [ 88%]
    tests/test_spawn_and_target.py::test_target_overlap_cannot_change_ball_trajectory[position2] PASSED [ 88%]
    tests/test_spawn_and_target.py::test_composition_rejects_robot_overlapping_default_ball_spawn PASSED [ 88%]
    tests/test_table.py::test_table_is_fixed_and_supports_ball PASSED        [ 88%]
    tests/test_table.py::test_leg_contacts_ball_below_tabletop[table_leg_front_left] PASSED [ 89%]
    tests/test_table.py::test_leg_contacts_ball_below_tabletop[table_leg_front_right] PASSED [ 89%]
    tests/test_table.py::test_leg_contacts_ball_below_tabletop[table_leg_back_left] PASSED [ 89%]
    tests/test_table.py::test_leg_contacts_ball_below_tabletop[table_leg_back_right] PASSED [ 89%]
    tests/test_table.py::test_space_under_table_is_open PASSED               [ 89%]
    tests/test_table.py::test_tabletop_contact_boundary[right-overlap] PASSED [ 89%]
    tests/test_table.py::test_tabletop_contact_boundary[right-clearance] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[left-overlap] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[left-clearance] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[back-overlap] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[back-clearance] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[front-overlap] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[front-clearance] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[underside-overlap] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[underside-clearance] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-ne-overlap] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-ne-clearance] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-nw-overlap] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-nw-clearance] PASSED [ 92%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-se-overlap] PASSED [ 92%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-se-clearance] PASSED [ 92%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-sw-overlap] PASSED [ 92%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-sw-clearance] PASSED [ 92%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0--1-table_leg_front_left] PASSED [ 92%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0--1-table_leg_front_right] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0--1-table_leg_back_left] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0--1-table_leg_back_right] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0-1-table_leg_front_left] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0-1-table_leg_front_right] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0-1-table_leg_back_left] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0-1-table_leg_back_right] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1--1-table_leg_front_left] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1--1-table_leg_front_right] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1--1-table_leg_back_left] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1--1-table_leg_back_right] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1-1-table_leg_front_left] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1-1-table_leg_front_right] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1-1-table_leg_back_left] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1-1-table_leg_back_right] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0--1-table_leg_front_left] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0--1-table_leg_front_right] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0--1-table_leg_back_left] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0--1-table_leg_back_right] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0-1-table_leg_front_left] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0-1-table_leg_front_right] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0-1-table_leg_back_left] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0-1-table_leg_back_right] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1--1-table_leg_front_left] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1--1-table_leg_front_right] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1--1-table_leg_back_left] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1--1-table_leg_back_right] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1-1-table_leg_front_left] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1-1-table_leg_front_right] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1-1-table_leg_back_left] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1-1-table_leg_back_right] PASSED [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[0.44-0] PASSED   [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[-0.44-0] PASSED  [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[0-0.34] PASSED   [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[0--0.34] PASSED  [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[0.44-0.34] PASSED [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[-0.44--0.34] PASSED [ 99%]
    tests/test_table.py::test_ball_settles_near_table_edges[-0.44-0.34] PASSED [ 99%]
    tests/test_table.py::test_ball_settles_near_table_edges[0.44--0.34] PASSED [ 99%]
    tests/test_table.py::test_ball_outside_table_falls_to_floor[0.51-0] PASSED [ 99%]
    tests/test_table.py::test_ball_outside_table_falls_to_floor[-0.51-0] PASSED [ 99%]
    tests/test_table.py::test_ball_outside_table_falls_to_floor[0-0.41] PASSED [ 99%]
    tests/test_table.py::test_ball_outside_table_falls_to_floor[0--0.41] PASSED [100%]
    
    =================================== FAILURES ===================================
    _______________ test_default_controller_period_advances_20_steps _______________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x1092a2580>
    
        def test_default_controller_period_advances_20_steps(sim):
            """40 ms requires 20 physics steps at the default 2 ms dt."""
            assert sim.model.opt.timestep == pytest.approx(0.002)
        
            initial_time = float(sim.data.time)
        
    >       sim.step_controller_period(period=0.04)
            ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E       AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:140: AttributeError
    _______________ test_one_controller_period_calls_mujoco_20_times _______________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x1081a2b00>
    monkeypatch = <_pytest.monkeypatch.MonkeyPatch object at 0x1081e2cf0>
    
        def test_one_controller_period_calls_mujoco_20_times(
            sim,
            monkeypatch,
        ):
            """Count real MuJoCo integrations, not only elapsed time."""
            original_step = mujoco.mj_step
            calls = []
        
            def recording_step(model, data):
                calls.append((model, data))
                return original_step(model, data)
        
            monkeypatch.setattr(
                mujoco,
                "mj_step",
                recording_step,
            )
        
    >       sim.step_controller_period(period=0.04)
            ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E       AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:167: AttributeError
    ________________ test_controller_period_holds_command_constant _________________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x106ddfdc0>
    monkeypatch = <_pytest.monkeypatch.MonkeyPatch object at 0x108f92d50>
    
        def test_controller_period_holds_command_constant(
            sim,
            monkeypatch,
        ):
            """Do not alter the selected command between physics steps."""
            command = sim.data.ctrl.copy()
            gripper_id = sim.model.actuator("actuator8").id
            command[gripper_id] = 255.0
        
            observed = []
            original_step = mujoco.mj_step
        
            def recording_step(model, data):
                observed.append(data.ctrl.copy())
                return original_step(model, data)
        
            monkeypatch.setattr(
                mujoco,
                "mj_step",
                recording_step,
            )
        
    >       sim.step_controller_period(
            ^^^^^^^^^^^^^^^^^^^^^^^^^^
                control=command,
                period=0.04,
            )
    E       AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:198: AttributeError
    _________________ test_none_control_retains_existing_commands __________________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x10932a140>
    
        def test_none_control_retains_existing_commands(sim):
            """The controller wrapper must preserve P0-006.05 semantics."""
            initial_controls = sim.data.ctrl.copy()
        
    >       sim.step_controller_period(
            ^^^^^^^^^^^^^^^^^^^^^^^^^^
                control=None,
                period=0.04,
            )
    E       AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:213: AttributeError
    ______________ test_controller_period_uses_current_model_timestep ______________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x10928bfc0>
    
        def test_controller_period_uses_current_model_timestep(sim):
            """Do not hardcode the original 0.002-second physics step."""
            sim.model.opt.timestep = 0.004
        
            initial_time = float(sim.data.time)
        
    >       sim.step_controller_period(period=0.04)
            ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E       AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:230: AttributeError
    _______________ test_invalid_period_preserves_state_and_control ________________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x1092dfb00>
    
        def test_invalid_period_preserves_state_and_control(sim):
            """Incompatible periods must not cause partial execution."""
            original_time = float(sim.data.time)
            original_qpos = sim.data.qpos.copy()
            original_ctrl = sim.data.ctrl.copy()
        
            with pytest.raises(ValueError):
    >           sim.step_controller_period(
                ^^^^^^^^^^^^^^^^^^^^^^^^^^
                    control=original_ctrl.copy(),
                    period=0.031,
                )
    E           AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:246: AttributeError
    __________________ test_invalid_period_does_not_call_physics ___________________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x10932a640>
    monkeypatch = <_pytest.monkeypatch.MonkeyPatch object at 0x108f90950>
    
        def test_invalid_period_does_not_call_physics(
            sim,
            monkeypatch,
        ):
            """Validate controller duration before any MuJoCo stepping."""
            def forbidden_step(*args, **kwargs):
                raise AssertionError(
                    "Invalid controller period reached MuJoCo"
                )
        
            monkeypatch.setattr(
                mujoco,
                "mj_step",
                forbidden_step,
            )
        
            with pytest.raises(ValueError):
    >           sim.step_controller_period(period=0.031)
                ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E           AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:279: AttributeError
    ___________ test_invalid_control_does_not_advance_controller_period ____________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x1092d8cc0>
    
        def test_invalid_control_does_not_advance_controller_period(sim):
            """The existing step() method must reject bad commands."""
            initial_time = float(sim.data.time)
            original_ctrl = sim.data.ctrl.copy()
        
            invalid = original_ctrl.copy()
            invalid[-1] = 9999.0
        
            with pytest.raises(ValueError):
    >           sim.step_controller_period(
                ^^^^^^^^^^^^^^^^^^^^^^^^^^
                    control=invalid,
                    period=0.04,
                )
    E           AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:291: AttributeError
    _______________________ test_missing_period_is_rejected ________________________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x1092de700>
    
        def test_missing_period_is_rejected(sim):
            """Require an explicit controller duration."""
            with pytest.raises(TypeError):
    >           sim.step_controller_period()
                ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E           AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:306: AttributeError
    ____________ test_repeated_controller_period_calls_accumulate_time _____________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x1092dad00>
    
        def test_repeated_controller_period_calls_accumulate_time(sim):
            """Multiple controller ticks advance consistent physics time."""
            initial_time = float(sim.data.time)
        
            for _ in range(3):
    >           sim.step_controller_period(period=0.04)
                ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E           AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:314: AttributeError
    ______________ test_physics_step_and_controller_step_can_be_mixed ______________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x106e21040>
    
        def test_physics_step_and_controller_step_can_be_mixed(sim):
            """Controller periods and raw steps use the same clock."""
            initial_time = float(sim.data.time)
        
            sim.step(n_substeps=5)
    >       sim.step_controller_period(period=0.04)
            ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E       AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:328: AttributeError
    _____________ test_reset_after_controller_period_restores_baseline _____________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x10927f780>
    
        def test_reset_after_controller_period_restores_baseline(sim):
            """Controller execution must remain compatible with reset."""
            initial_time = float(sim.data.time)
            initial_qpos = sim.data.qpos.copy()
        
    >       sim.step_controller_period(period=0.04)
            ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E       AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:342: AttributeError
    _____________ test_controller_period_preserves_model_data_identity _____________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x108215440>
    
        def test_controller_period_preserves_model_data_identity(sim):
            """No compilation or replacement should occur."""
            original_model = sim.model
            original_data = sim.data
        
    >       sim.step_controller_period(period=0.04)
            ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E       AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:359: AttributeError
    _____________ test_two_simulators_have_independent_controller_time _____________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x1092d8340>
    
        def test_two_simulators_have_independent_controller_time(sim):
            """A controller tick in one instance leaves the other untouched."""
            source = os.environ["ROBOT_ARM_PANDA_XML"]
            other = Simulator(Path(source))
            other.load()
        
            try:
                other_time = float(other.data.time)
        
    >           sim.step_controller_period(period=0.04)
                ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E           AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:374: AttributeError
    ________________ test_controller_period_before_load_is_rejected ________________
    
        def test_controller_period_before_load_is_rejected():
            """The wrapper cannot operate before model loading."""
            sim = Simulator("unused.xml")
        
            with pytest.raises(SimulatorLifecycleError):
    >           sim.step_controller_period(period=0.04)
                ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E           AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:387: AttributeError
    ________________ test_controller_period_after_close_is_rejected ________________
    
    sim = <robot_arm_sim.simulator.Simulator object at 0x10933cb00>
    
        def test_controller_period_after_close_is_rejected(sim):
            """Closed simulators cannot resume execution."""
            sim.close()
        
            with pytest.raises(SimulatorLifecycleError):
    >           sim.step_controller_period(period=0.04)
                ^^^^^^^^^^^^^^^^^^^^^^^^^^
    E           AttributeError: 'Simulator' object has no attribute 'step_controller_period'
    
    tests/test_simulator_controller_period.py:395: AttributeError
    =========================== short test summary info ============================
    FAILED tests/test_simulator_controller_period.py::test_default_controller_period_advances_20_steps
    FAILED tests/test_simulator_controller_period.py::test_one_controller_period_calls_mujoco_20_times
    FAILED tests/test_simulator_controller_period.py::test_controller_period_holds_command_constant
    FAILED tests/test_simulator_controller_period.py::test_none_control_retains_existing_commands
    FAILED tests/test_simulator_controller_period.py::test_controller_period_uses_current_model_timestep
    FAILED tests/test_simulator_controller_period.py::test_invalid_period_preserves_state_and_control
    FAILED tests/test_simulator_controller_period.py::test_invalid_period_does_not_call_physics
    FAILED tests/test_simulator_controller_period.py::test_invalid_control_does_not_advance_controller_period
    FAILED tests/test_simulator_controller_period.py::test_missing_period_is_rejected
    FAILED tests/test_simulator_controller_period.py::test_repeated_controller_period_calls_accumulate_time
    FAILED tests/test_simulator_controller_period.py::test_physics_step_and_controller_step_can_be_mixed
    FAILED tests/test_simulator_controller_period.py::test_reset_after_controller_period_restores_baseline
    FAILED tests/test_simulator_controller_period.py::test_controller_period_preserves_model_data_identity
    FAILED tests/test_simulator_controller_period.py::test_two_simulators_have_independent_controller_time
    FAILED tests/test_simulator_controller_period.py::test_controller_period_before_load_is_rejected
    FAILED tests/test_simulator_controller_period.py::test_controller_period_after_close_is_rejected
    ======================== 16 failed, 586 passed in 3.96s ========================


## Test run: 2026-10-24T22:08:28+02:00

Exit status: 0

    ============================= test session starts ==============================
    platform darwin -- Python 3.14.6, pytest-9.1.1, pluggy-1.7.0 -- /Users/catherinepemblington/Documents/GitHub/Robot_arm/.venv/bin/python
    cachedir: .pytest_cache
    rootdir: /Users/catherinepemblington/Documents/GitHub/Robot_arm
    configfile: pyproject.toml
    collecting ... collected 602 items
    
    tests/test_ball.py::test_ball_has_uniform_solid_sphere_inertia[demo] PASSED [  0%]
    tests/test_ball.py::test_ball_has_uniform_solid_sphere_inertia[composed] PASSED [  0%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[demo-x] PASSED [  0%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[demo-y] PASSED [  0%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[demo-z] PASSED [  0%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[composed-x] PASSED [  0%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[composed-y] PASSED [  1%]
    tests/test_ball.py::test_ball_acceleration_matches_force_and_mass[composed-z] PASSED [  1%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[demo-x] PASSED [  1%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[demo-y] PASSED [  1%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[demo-z] PASSED [  1%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[composed-x] PASSED [  1%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[composed-y] PASSED [  2%]
    tests/test_ball.py::test_ball_angular_acceleration_matches_torque_and_inertia[composed-z] PASSED [  2%]
    tests/test_ball.py::test_ball_uses_specified_sliding_friction_in_table_contact[demo] PASSED [  2%]
    tests/test_ball.py::test_ball_uses_specified_sliding_friction_in_table_contact[composed] PASSED [  2%]
    tests/test_ball.py::test_ball_free_joint_has_six_unconstrained_velocity_dofs[demo] PASSED [  2%]
    tests/test_ball.py::test_ball_free_joint_has_six_unconstrained_velocity_dofs[composed] PASSED [  2%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[demo-rest] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[demo-horizontal] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[demo-upward] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[demo-downward] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[composed-rest] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[composed-horizontal] PASSED [  3%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[composed-upward] PASSED [  4%]
    tests/test_ball.py::test_free_ball_follows_ballistic_motion[composed-downward] PASSED [  4%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[demo-x] PASSED [  4%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[demo-y] PASSED [  4%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[demo-z] PASSED [  4%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[composed-x] PASSED [  4%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[composed-y] PASSED [  5%]
    tests/test_ball.py::test_free_ball_keeps_spinning_without_applied_torque[composed-z] PASSED [  5%]
    tests/test_ball.py::test_table_side_collision_changes_free_ball_momentum[demo-left] PASSED [  5%]
    tests/test_ball.py::test_table_side_collision_changes_free_ball_momentum[demo-right] PASSED [  5%]
    tests/test_ball.py::test_table_side_collision_changes_free_ball_momentum[composed-left] PASSED [  5%]
    tests/test_ball.py::test_table_side_collision_changes_free_ball_momentum[composed-right] PASSED [  5%]
    tests/test_cli.py::test_robot_model_requires_robot_flag PASSED           [  6%]
    tests/test_cli.py::test_robot_mode_reports_missing_model_path PASSED     [  6%]
    tests/test_config.py::test_default_config_resolves_packaged_model_and_values PASSED [  6%]
    tests/test_config.py::test_load_config_rejects_invalid_timing[0-0.04-timestep must be greater than zero] PASSED [  6%]
    tests/test_config.py::test_load_config_rejects_invalid_timing[true-0.04-timestep must be a number] PASSED [  6%]
    tests/test_config.py::test_load_config_rejects_invalid_timing[0.002-0.031-integer multiple] PASSED [  6%]
    tests/test_config.py::test_load_config_resolves_relative_model_path PASSED [  7%]
    tests/test_config.py::test_load_config_rejects_absolute_model_path PASSED [  7%]
    tests/test_config.py::test_load_config_rejects_missing_model PASSED      [  7%]
    tests/test_config.py::test_apply_config_applies_timestep_and_scene_positions PASSED [  7%]
    tests/test_config.py::test_apply_config_checks_joint_and_actuator_limits[{"finger_joint" = 1.5}-{}-outside its range] PASSED [  7%]
    tests/test_config.py::test_apply_config_checks_joint_and_actuator_limits[{}-{"gripper" = 1.5}-outside its range] PASSED [  7%]
    tests/test_config.py::test_apply_config_checks_joint_and_actuator_limits[{"unknown_joint" = 0.5}-{}-joint does not exist] PASSED [  8%]
    tests/test_config.py::test_apply_config_checks_joint_and_actuator_limits[{}-{"unknown_actuator" = 0.5}-actuator does not exist] PASSED [  8%]
    tests/test_contact_filters.py::test_robot_collision_masks_allow_scene_contacts_and_visuals_are_disabled PASSED [  8%]
    tests/test_contact_filters.py::test_ball_contacts_every_hand_and_finger_collider[hand] PASSED [  8%]
    tests/test_contact_filters.py::test_ball_contacts_every_hand_and_finger_collider[left_finger] PASSED [  8%]
    tests/test_contact_filters.py::test_ball_contacts_every_hand_and_finger_collider[right_finger] PASSED [  8%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[floor-hand] PASSED [  9%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[floor-left_finger] PASSED [  9%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[floor-right_finger] PASSED [  9%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[table_top-hand] PASSED [  9%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[table_top-left_finger] PASSED [  9%]
    tests/test_contact_filters.py::test_supports_contact_hand_and_fingers[table_top-right_finger] PASSED [  9%]
    tests/test_contact_filters.py::test_original_base_link_exclusion_is_preserved PASSED [ 10%]
    tests/test_floor.py::test_floor_is_fixed_at_world_zero_with_debug_material PASSED [ 10%]
    tests/test_floor.py::test_floor_contact_boundary[overlap] PASSED         [ 10%]
    tests/test_floor.py::test_floor_contact_boundary[clearance] PASSED       [ 10%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[0.8-0] PASSED [ 10%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[-0.8-0] PASSED [ 10%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[0-0.8] PASSED [ 11%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[0--0.8] PASSED [ 11%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[2.5-0] PASSED [ 11%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[-2.5-0] PASSED [ 11%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[0-2.5] PASSED [ 11%]
    tests/test_floor.py::test_floor_catches_ball_even_beyond_visible_patch[0--2.5] PASSED [ 11%]
    tests/test_model_addresses.py::test_correct_joint_address_dimensions PASSED [ 12%]
    tests/test_model_addresses.py::test_qpos_and_qvel_addresses_are_not_confused PASSED [ 12%]
    tests/test_model_addresses.py::test_all_joint_slices_within_bounds PASSED [ 12%]
    tests/test_model_addresses.py::test_free_joint_slices_read_correct_dimensions PASSED [ 12%]
    tests/test_model_addresses.py::test_different_model_is_rejected PASSED   [ 12%]
    tests/test_model_addresses.py::test_wrong_joint_identifier_is_rejected PASSED [ 12%]
    tests/test_model_addresses.py::test_cross_wired_joint_identifier_is_rejected PASSED [ 13%]
    tests/test_model_addresses.py::test_negative_identifier_is_rejected PASSED [ 13%]
    tests/test_model_addresses.py::test_boolean_identifier_is_rejected PASSED [ 13%]
    tests/test_model_addresses.py::test_wrong_actuator_identifier_is_rejected PASSED [ 13%]
    tests/test_model_addresses.py::test_cache_rejects_mutation PASSED        [ 13%]
    tests/test_model_addresses.py::test_input_mapping_mutation_does_not_change_cache PASSED [ 13%]
    tests/test_model_addresses.py::test_missing_cached_joint_rejected PASSED [ 14%]
    tests/test_model_addresses.py::test_cache_build_does_not_mutate_model_or_state PASSED [ 14%]
    tests/test_model_addresses.py::test_correct_actuator_control_index PASSED [ 14%]
    tests/test_model_addresses.py::test_all_four_joint_types_have_correct_widths PASSED [ 14%]
    tests/test_model_addresses.py::test_all_joint_slices_cover_exact_state_dimensions PASSED [ 14%]
    tests/test_model_addresses.py::test_free_joint_is_not_treated_as_six_qpos_values PASSED [ 14%]
    tests/test_model_addresses.py::test_recompiled_equivalent_model_rejects_old_cache PASSED [ 15%]
    tests/test_model_addresses.py::test_same_numeric_joint_id_wrong_name_is_rejected PASSED [ 15%]
    tests/test_model_addresses.py::test_wrong_body_id_is_rejected PASSED     [ 15%]
    tests/test_model_addresses.py::test_wrong_site_id_is_rejected PASSED     [ 15%]
    tests/test_model_addresses.py::test_negative_body_id_rejected PASSED     [ 15%]
    tests/test_model_addresses.py::test_cached_mapping_is_read_only PASSED   [ 15%]
    tests/test_model_addresses.py::test_missing_actuator_lookup_fails PASSED [ 16%]
    tests/test_model_addresses.py::test_independent_caches_do_not_share_mapping_storage PASSED [ 16%]
    tests/test_model_addresses.py::test_panda_joint_and_actuator_address_cache PASSED [ 16%]
    tests/test_model_addresses.py::test_panda_cache_isolation_across_instances PASSED [ 16%]
    tests/test_model_diagnostics.py::test_real_panda_diagnostics_pass PASSED [ 16%]
    tests/test_model_diagnostics.py::test_diagnostic_metadata_matches_compiled_model PASSED [ 16%]
    tests/test_model_diagnostics.py::test_panda_expected_dimensions PASSED   [ 17%]
    tests/test_model_diagnostics.py::test_report_contains_all_required_stages PASSED [ 17%]
    tests/test_model_diagnostics.py::test_source_path_is_identification_only PASSED [ 17%]
    tests/test_model_diagnostics.py::test_blank_source_path_rejected[] PASSED [ 17%]
    tests/test_model_diagnostics.py::test_blank_source_path_rejected[   ] PASSED [ 17%]
    tests/test_model_diagnostics.py::test_invalid_model_argument_rejected[None] PASSED [ 17%]
    tests/test_model_diagnostics.py::test_invalid_model_argument_rejected[5] PASSED [ 18%]
    tests/test_model_diagnostics.py::test_invalid_model_argument_rejected[model] PASSED [ 18%]
    tests/test_model_diagnostics.py::test_invalid_source_type_rejected PASSED [ 18%]
    tests/test_model_diagnostics.py::test_missing_required_name_blocks_address_cache PASSED [ 18%]
    tests/test_model_diagnostics.py::test_disabled_joint_limit_fails_overall PASSED [ 18%]
    tests/test_model_diagnostics.py::test_nonfinite_actuator_range_is_reported PASSED [ 18%]
    tests/test_model_diagnostics.py::test_invalid_timestep_is_reported PASSED [ 19%]
    tests/test_model_diagnostics.py::test_diagnostic_report_is_immutable PASSED [ 19%]
    tests/test_model_diagnostics.py::test_report_is_snapshot_not_live_model_view PASSED [ 19%]
    tests/test_model_diagnostics.py::test_repeated_diagnostics_are_deterministic PASSED [ 19%]
    tests/test_model_diagnostics.py::test_text_report_contains_required_counts PASSED [ 19%]
    tests/test_model_diagnostics.py::test_failure_text_contains_actionable_error PASSED [ 19%]
    tests/test_model_diagnostics.py::test_blocked_stage_does_not_appear_as_pass PASSED [ 20%]
    tests/test_model_diagnostics.py::test_no_checks_is_not_a_success PASSED  [ 20%]
    tests/test_model_diagnostics.py::test_invalid_report_type_rejected PASSED [ 20%]
    tests/test_model_diagnostics.py::test_diagnostics_do_not_mutate_model_or_data PASSED [ 20%]
    tests/test_model_diagnostics.py::test_failure_diagnostics_do_not_mutate_model PASSED [ 20%]
    tests/test_model_diagnostics.py::test_formatter_does_not_print PASSED    [ 20%]
    tests/test_model_diagnostics.py::test_diagnostics_never_call_mj_step PASSED [ 21%]
    tests/test_model_diagnostics.py::test_invalid_physics_timestep_never_passes[nan] PASSED [ 21%]
    tests/test_model_diagnostics.py::test_invalid_physics_timestep_never_passes[inf] PASSED [ 21%]
    tests/test_model_diagnostics.py::test_invalid_physics_timestep_never_passes[-inf] PASSED [ 21%]
    tests/test_model_diagnostics.py::test_invalid_physics_timestep_never_passes[0.0] PASSED [ 21%]
    tests/test_model_diagnostics.py::test_invalid_physics_timestep_never_passes[-0.002] PASSED [ 21%]
    tests/test_model_diagnostics.py::test_malformed_model_reports_multiple_failures PASSED [ 22%]
    tests/test_model_diagnostics.py::test_unexpected_programming_error_is_not_masked PASSED [ 22%]
    tests/test_model_diagnostics.py::test_report_is_not_revalidated_when_formatted PASSED [ 22%]
    tests/test_model_ids.py::test_resolves_existing_names[object_type0-test_joint] PASSED [ 22%]
    tests/test_model_ids.py::test_resolves_existing_names[object_type1-test_actuator] PASSED [ 22%]
    tests/test_model_ids.py::test_resolves_existing_names[object_type2-test_body] PASSED [ 22%]
    tests/test_model_ids.py::test_resolves_existing_names[object_type3-test_site] PASSED [ 23%]
    tests/test_model_ids.py::test_missing_name_is_rejected[object_type0] PASSED [ 23%]
    tests/test_model_ids.py::test_missing_name_is_rejected[object_type1] PASSED [ 23%]
    tests/test_model_ids.py::test_missing_name_is_rejected[object_type2] PASSED [ 23%]
    tests/test_model_ids.py::test_missing_name_is_rejected[object_type3] PASSED [ 23%]
    tests/test_model_ids.py::test_wrong_object_category_rejected PASSED      [ 23%]
    tests/test_model_ids.py::test_blank_names_rejected[] PASSED              [ 24%]
    tests/test_model_ids.py::test_blank_names_rejected[ ] PASSED             [ 24%]
    tests/test_model_ids.py::test_blank_names_rejected[\t] PASSED            [ 24%]
    tests/test_model_ids.py::test_invalid_model_type_rejected PASSED         [ 24%]
    tests/test_model_ids.py::test_invalid_object_type_rejected PASSED        [ 24%]
    tests/test_model_ids.py::test_unsupported_object_category_rejected PASSED [ 24%]
    tests/test_model_ids.py::test_complete_custom_schema PASSED              [ 25%]
    tests/test_model_ids.py::test_missing_components_reported_together PASSED [ 25%]
    tests/test_model_ids.py::test_duplicate_names_rejected PASSED            [ 25%]
    tests/test_model_ids.py::test_no_physics_step_occurs PASSED              [ 25%]
    tests/test_model_ids.py::test_default_panda_schema_rejects_incomplete_model PASSED [ 25%]
    tests/test_model_ids.py::test_same_name_in_different_object_categories PASSED [ 25%]
    tests/test_model_ids.py::test_name_matching_is_exact[Test_Joint] PASSED  [ 26%]
    tests/test_model_ids.py::test_name_matching_is_exact[TEST_JOINT] PASSED  [ 26%]
    tests/test_model_ids.py::test_name_matching_is_exact[test_Joint] PASSED  [ 26%]
    tests/test_model_ids.py::test_name_matching_is_exact[test_joint ] PASSED [ 26%]
    tests/test_model_ids.py::test_name_matching_is_exact[ test_joint] PASSED [ 26%]
    tests/test_model_ids.py::test_name_matching_is_exact[test_joint\n] PASSED [ 26%]
    tests/test_model_ids.py::test_duplicate_names_across_joint_groups_rejected PASSED [ 27%]
    tests/test_model_ids.py::test_duplicate_names_across_actuator_groups_rejected PASSED [ 27%]
    tests/test_model_ids.py::test_duplicate_names_across_body_groups_rejected PASSED [ 27%]
    tests/test_model_ids.py::test_empty_schema_is_rejected_for_full_validation PASSED [ 27%]
    tests/test_model_ids.py::test_model_ids_recomputed_when_joint_order_changes PASSED [ 27%]
    tests/test_model_ids.py::test_validation_does_not_modify_compiled_model PASSED [ 27%]
    tests/test_model_ids.py::test_failed_validation_does_not_mutate_model PASSED [ 28%]
    tests/test_model_ids.py::test_resolved_result_does_not_depend_on_later_lookups PASSED [ 28%]
    tests/test_model_ids.py::test_missing_site_does_not_fallback_to_body PASSED [ 28%]
    tests/test_model_ids.py::test_unexpected_schema_type_is_rejected PASSED  [ 28%]
    tests/test_model_instance.py::test_panda_multi_instance_state_isolation PASSED [ 28%]
    tests/test_model_instance.py::test_instance_contains_correct_types PASSED [ 28%]
    tests/test_model_instance.py::test_separate_instances_have_distinct_models PASSED [ 29%]
    tests/test_model_instance.py::test_initial_states_are_equivalent PASSED  [ 29%]
    tests/test_model_instance.py::test_joint_positions_are_independent PASSED [ 29%]
    tests/test_model_instance.py::test_joint_velocities_are_independent PASSED [ 29%]
    tests/test_model_instance.py::test_actuator_commands_are_independent PASSED [ 29%]
    tests/test_model_instance.py::test_mutable_model_properties_are_independent PASSED [ 29%]
    tests/test_model_instance.py::test_stepping_one_instance_does_not_step_another PASSED [ 30%]
    tests/test_model_instance.py::test_repeated_stepping_remains_isolated PASSED [ 30%]
    tests/test_model_instance.py::test_attribute_reassignment_is_rejected PASSED [ 30%]
    tests/test_model_instance.py::test_instance_keeps_model_and_data_alive PASSED [ 30%]
    tests/test_model_instance.py::test_multiple_instances_remain_usable PASSED [ 30%]
    tests/test_model_instance.py::test_loading_has_no_physics_time_advance PASSED [ 30%]
    tests/test_model_instance.py::test_source_model_remains_unchanged PASSED [ 31%]
    tests/test_model_instance.py::test_invalid_file_propagates_loader_error PASSED [ 31%]
    tests/test_model_instance.py::test_real_panda_instances_are_independent PASSED [ 31%]
    tests/test_model_instance.py::test_public_constructor_rejected PASSED    [ 31%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[model] PASSED [ 31%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[data] PASSED [ 31%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[_model] PASSED [ 32%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[_data] PASSED [ 32%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[_sealed] PASSED [ 32%]
    tests/test_model_instance.py::test_all_ordinary_attribute_writes_rejected[extra] PASSED [ 32%]
    tests/test_model_instance.py::test_ownership_attribute_deletion_rejected[model] PASSED [ 32%]
    tests/test_model_instance.py::test_ownership_attribute_deletion_rejected[data] PASSED [ 32%]
    tests/test_model_instance.py::test_ownership_attribute_deletion_rejected[_model] PASSED [ 33%]
    tests/test_model_instance.py::test_ownership_attribute_deletion_rejected[_data] PASSED [ 33%]
    tests/test_model_instance.py::test_ownership_attribute_deletion_rejected[_sealed] PASSED [ 33%]
    tests/test_model_instance.py::test_shallow_copy_is_rejected PASSED       [ 33%]
    tests/test_model_instance.py::test_deep_copy_is_rejected PASSED          [ 33%]
    tests/test_model_instance.py::test_pickle_roundtrip_is_rejected PASSED   [ 33%]
    tests/test_model_instance.py::test_independent_arrays_do_not_share_memory PASSED [ 34%]
    tests/test_model_instance.py::test_state_mutation_does_not_cross_instances PASSED [ 34%]
    tests/test_model_instance.py::test_reverse_direction_isolation PASSED    [ 34%]
    tests/test_model_instance.py::test_mutable_model_arrays_are_independent PASSED [ 34%]
    tests/test_model_instance.py::test_repeated_creation_does_not_reuse_models PASSED [ 34%]
    tests/test_model_instance.py::test_gc_does_not_corrupt_surviving_instance PASSED [ 34%]
    tests/test_model_instance.py::test_failed_load_produces_no_instance PASSED [ 35%]
    tests/test_model_instance.py::test_failed_scene_creation_produces_no_instance PASSED [ 35%]
    tests/test_model_instance.py::test_invalid_internal_pair_types_rejected PASSED [ 35%]
    tests/test_model_instance.py::test_source_file_hash_remains_unchanged PASSED [ 35%]
    tests/test_model_instance.py::test_parallel_interleaved_simulations_isolated PASSED [ 35%]
    tests/test_model_instance.py::test_subclass_factory_is_rejected PASSED   [ 35%]
    tests/test_model_limits.py::test_real_panda_limits_valid PASSED          [ 36%]
    tests/test_model_limits.py::test_all_home_values_inside_bounds PASSED    [ 36%]
    tests/test_model_limits.py::test_invalid_joint_defaults_rejected[nan] PASSED [ 36%]
    tests/test_model_limits.py::test_invalid_joint_defaults_rejected[inf] PASSED [ 36%]
    tests/test_model_limits.py::test_invalid_joint_defaults_rejected[-inf] PASSED [ 36%]
    tests/test_model_limits.py::test_invalid_joint_defaults_rejected[True] PASSED [ 36%]
    tests/test_model_limits.py::test_invalid_joint_defaults_rejected[invalid] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[nan] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[inf] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[-inf] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[-1.0] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[256.0] PASSED [ 37%]
    tests/test_model_limits.py::test_invalid_gripper_commands_rejected[True] PASSED [ 38%]
    tests/test_model_limits.py::test_joint_limit_disabled_rejected PASSED    [ 38%]
    tests/test_model_limits.py::test_actuator_limit_disabled_rejected PASSED [ 38%]
    tests/test_model_limits.py::test_nonfinite_joint_bound_rejected[nan] PASSED [ 38%]
    tests/test_model_limits.py::test_nonfinite_joint_bound_rejected[inf] PASSED [ 38%]
    tests/test_model_limits.py::test_nonfinite_joint_bound_rejected[-inf] PASSED [ 38%]
    tests/test_model_limits.py::test_reversed_actuator_range_rejected PASSED [ 39%]
    tests/test_model_limits.py::test_corrupted_home_position_rejected PASSED [ 39%]
    tests/test_model_limits.py::test_gripper_range_is_not_metres PASSED      [ 39%]
    tests/test_model_limits.py::test_validator_preserves_model PASSED        [ 39%]
    tests/test_model_loader.py::test_valid_model_loads PASSED                [ 39%]
    tests/test_model_loader.py::test_string_path_is_supported PASSED         [ 39%]
    tests/test_model_loader.py::test_relative_path_is_supported PASSED       [ 40%]
    tests/test_model_loader.py::test_missing_file_has_useful_error PASSED    [ 40%]
    tests/test_model_loader.py::test_directory_is_rejected PASSED            [ 40%]
    tests/test_model_loader.py::test_empty_paths_are_rejected[] PASSED       [ 40%]
    tests/test_model_loader.py::test_empty_paths_are_rejected[ ] PASSED      [ 40%]
    tests/test_model_loader.py::test_empty_paths_are_rejected[\t] PASSED     [ 40%]
    tests/test_model_loader.py::test_empty_paths_are_rejected[\n] PASSED     [ 41%]
    tests/test_model_loader.py::test_invalid_path_types_are_rejected[None] PASSED [ 41%]
    tests/test_model_loader.py::test_invalid_path_types_are_rejected[123] PASSED [ 41%]
    tests/test_model_loader.py::test_invalid_path_types_are_rejected[1.5] PASSED [ 41%]
    tests/test_model_loader.py::test_invalid_path_types_are_rejected[path3] PASSED [ 41%]
    tests/test_model_loader.py::test_invalid_path_types_are_rejected[path4] PASSED [ 41%]
    tests/test_model_loader.py::test_malformed_xml_is_rejected PASSED        [ 42%]
    tests/test_model_loader.py::test_empty_file_is_rejected PASSED           [ 42%]
    tests/test_model_loader.py::test_invalid_mujoco_element_is_rejected PASSED [ 42%]
    tests/test_model_loader.py::test_unknown_joint_reference_is_rejected PASSED [ 42%]
    tests/test_model_loader.py::test_missing_mesh_dependency_is_rejected PASSED [ 42%]
    tests/test_model_loader.py::test_model_loading_preserves_original_error PASSED [ 42%]
    tests/test_model_loader.py::test_model_file_is_not_modified PASSED       [ 43%]
    tests/test_model_loader.py::test_loading_produces_independent_states PASSED [ 43%]
    tests/test_model_loader.py::test_loading_does_not_step_physics PASSED    [ 43%]
    tests/test_model_loader.py::test_invalid_content_fails_cleanly[<not_mujoco/>] PASSED [ 43%]
    tests/test_model_loader.py::test_invalid_content_fails_cleanly[<mujoco><worldbody>] PASSED [ 43%]
    tests/test_model_loader.py::test_invalid_content_fails_cleanly[<mujoco><worldbody/><extra/></mujoco>] PASSED [ 43%]
    tests/test_model_loader.py::test_invalid_content_fails_cleanly[\x00] PASSED [ 44%]
    tests/test_model_structure.py::test_valid_structure_passes PASSED        [ 44%]
    tests/test_model_structure.py::test_correct_total_state_dimensions PASSED [ 44%]
    tests/test_model_structure.py::test_missing_arm_joint_rejected PASSED    [ 44%]
    tests/test_model_structure.py::test_extra_joint_rejected PASSED          [ 44%]
    tests/test_model_structure.py::test_wrong_finger_joint_type_rejected[hinge] PASSED [ 44%]
    tests/test_model_structure.py::test_wrong_finger_joint_type_rejected[ball] PASSED [ 45%]
    tests/test_model_structure.py::test_ball_must_be_free_joint[hinge] PASSED [ 45%]
    tests/test_model_structure.py::test_ball_must_be_free_joint[slide] PASSED [ 45%]
    tests/test_model_structure.py::test_ball_must_be_free_joint[ball] PASSED [ 45%]
    tests/test_model_structure.py::test_free_joint_attached_to_wrong_body_rejected PASSED [ 45%]
    tests/test_model_structure.py::test_missing_actuator_rejected[actuator1] PASSED [ 45%]
    tests/test_model_structure.py::test_missing_actuator_rejected[actuator4] PASSED [ 46%]
    tests/test_model_structure.py::test_missing_actuator_rejected[actuator7] PASSED [ 46%]
    tests/test_model_structure.py::test_missing_actuator_rejected[actuator8] PASSED [ 46%]
    tests/test_model_structure.py::test_extra_actuator_rejected PASSED       [ 46%]
    tests/test_model_structure.py::test_wrong_model_argument_rejected PASSED [ 46%]
    tests/test_model_structure.py::test_validation_is_read_only PASSED       [ 46%]
    tests/test_model_structure.py::test_failure_does_not_modify_state PASSED [ 47%]
    tests/test_model_structure.py::test_real_panda_structure PASSED          [ 47%]
    tests/test_model_structure.py::test_challenge_extra_joint_with_valid_required_names PASSED [ 47%]
    tests/test_model_structure.py::test_challenge_extra_joint_failure_is_read_only PASSED [ 47%]
    tests/test_model_structure.py::test_challenge_ball_free_joint_on_wrong_body PASSED [ 47%]
    tests/test_model_structure.py::test_challenge_wrong_ball_joint_does_not_advance_physics PASSED [ 47%]
    tests/test_model_structure.py::test_challenge_arm_joint_type_changed_with_same_dimensions PASSED [ 48%]
    tests/test_robot_placement.py::test_base_and_pedestal_stay_rigid PASSED  [ 48%]
    tests/test_robot_placement.py::test_movable_base_is_rejected[<freejoint/>] PASSED [ 48%]
    tests/test_robot_placement.py::test_movable_base_is_rejected[<joint type="hinge"/>] PASSED [ 48%]
    tests/test_robot_placement.py::test_fixed_geometry_penetration_is_rejected[0.65 0 -0.025-table_top] PASSED [ 48%]
    tests/test_robot_placement.py::test_fixed_geometry_penetration_is_rejected[0.26 -0.29 -0.275-table_leg_front_left] PASSED [ 48%]
    tests/test_robot_placement.py::test_fixed_geometry_penetration_is_rejected[1.5 0 -0.5-floor] PASSED [ 49%]
    tests/test_robot_placement.py::test_fixed_geometry_penetration_is_rejected[0 0 -0.1-robot_pedestal] PASSED [ 49%]
    tests/test_robot_placement.py::test_visual_only_geometry_does_not_block_reset PASSED [ 49%]
    tests/test_robot_placement.py::test_changed_reset_pose_is_checked PASSED [ 49%]
    tests/test_robot_placement.py::test_panda_home_and_nearby_sweeps_clear_supports PASSED [ 49%]
    tests/test_robot_placement.py::test_panda_base_stays_fixed_and_home_remains_clear PASSED [ 49%]
    tests/test_scene_settings.py::test_explicit_physics_options_survive_configuration[demo] PASSED [ 50%]
    tests/test_scene_settings.py::test_explicit_physics_options_survive_configuration[composed] PASSED [ 50%]
    tests/test_scene_settings.py::test_fixed_cameras_frame_table_ball_goal_and_robot_envelope[demo-overview] PASSED [ 50%]
    tests/test_scene_settings.py::test_fixed_cameras_frame_table_ball_goal_and_robot_envelope[demo-top_down] PASSED [ 50%]
    tests/test_scene_settings.py::test_fixed_cameras_frame_table_ball_goal_and_robot_envelope[composed-overview] PASSED [ 50%]
    tests/test_scene_settings.py::test_fixed_cameras_frame_table_ball_goal_and_robot_envelope[composed-top_down] PASSED [ 50%]
    tests/test_scene_settings.py::test_debug_lights_and_colours_are_distinct[demo] PASSED [ 50%]
    tests/test_scene_settings.py::test_debug_lights_and_colours_are_distinct[composed] PASSED [ 51%]
    tests/test_scene_units.py::test_gravity_is_earth_like_and_points_down PASSED [ 51%]
    tests/test_scene_units.py::test_scene_uses_radians_and_si_scale PASSED   [ 51%]
    tests/test_scene_units.py::test_robot_scene_keeps_gravity PASSED         [ 51%]
    tests/test_scene_units.py::test_free_fall_acceleration_matches_gravity PASSED [ 51%]
    tests/test_simulation.py::test_build_scene_attaches_robot_model PASSED   [ 51%]
    tests/test_simulation.py::test_build_scene_rejects_missing_model PASSED  [ 52%]
    tests/test_simulation.py::test_build_scene_rejects_invalid_xml PASSED    [ 52%]
    tests/test_simulation.py::test_build_scene_requires_home_keyframe PASSED [ 52%]
    tests/test_simulation.py::test_build_scene_copies_articulated_joint_types_from_home_keyframe PASSED [ 52%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.002-0.002-1] PASSED [ 52%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.01-0.002-5] PASSED [ 52%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.04-0.002-20] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.08-0.002-40] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.03-0.003-10] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_compatible_periods_have_expected_substep_counts[0.001-0.001-1] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_noninteger_or_short_periods_are_rejected[0.031-0.002] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_noninteger_or_short_periods_are_rejected[0.025-0.004] PASSED [ 53%]
    tests/test_simulator_controller_period.py::test_noninteger_or_short_periods_are_rejected[0.001-0.002] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_noninteger_or_short_periods_are_rejected[0.0001-0.002] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_noninteger_or_short_periods_are_rejected[0.0401-0.002] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_invalid_period_values_are_rejected[0.0] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_invalid_period_values_are_rejected[-0.04] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_invalid_period_values_are_rejected[nan] PASSED [ 54%]
    tests/test_simulator_controller_period.py::test_invalid_period_values_are_rejected[inf] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_values_are_rejected[-inf] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[True] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[False] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[None] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[0.04] PASSED [ 55%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[period4] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_invalid_period_types_are_rejected[period5] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_invalid_physics_timestep_is_rejected[0.0] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_invalid_physics_timestep_is_rejected[-0.002] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_invalid_physics_timestep_is_rejected[nan] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_invalid_physics_timestep_is_rejected[inf] PASSED [ 56%]
    tests/test_simulator_controller_period.py::test_float_representation_noise_is_accepted PASSED [ 57%]
    tests/test_simulator_controller_period.py::test_large_finite_but_nonrepresentable_ratio_rejected PASSED [ 57%]
    tests/test_simulator_controller_period.py::test_default_controller_period_advances_20_steps PASSED [ 57%]
    tests/test_simulator_controller_period.py::test_one_controller_period_calls_mujoco_20_times PASSED [ 57%]
    tests/test_simulator_controller_period.py::test_controller_period_holds_command_constant PASSED [ 57%]
    tests/test_simulator_controller_period.py::test_none_control_retains_existing_commands PASSED [ 57%]
    tests/test_simulator_controller_period.py::test_controller_period_uses_current_model_timestep PASSED [ 58%]
    tests/test_simulator_controller_period.py::test_invalid_period_preserves_state_and_control PASSED [ 58%]
    tests/test_simulator_controller_period.py::test_invalid_period_does_not_call_physics PASSED [ 58%]
    tests/test_simulator_controller_period.py::test_invalid_control_does_not_advance_controller_period PASSED [ 58%]
    tests/test_simulator_controller_period.py::test_missing_period_is_rejected PASSED [ 58%]
    tests/test_simulator_controller_period.py::test_repeated_controller_period_calls_accumulate_time PASSED [ 58%]
    tests/test_simulator_controller_period.py::test_physics_step_and_controller_step_can_be_mixed PASSED [ 59%]
    tests/test_simulator_controller_period.py::test_reset_after_controller_period_restores_baseline PASSED [ 59%]
    tests/test_simulator_controller_period.py::test_controller_period_preserves_model_data_identity PASSED [ 59%]
    tests/test_simulator_controller_period.py::test_two_simulators_have_independent_controller_time PASSED [ 59%]
    tests/test_simulator_controller_period.py::test_controller_period_before_load_is_rejected PASSED [ 59%]
    tests/test_simulator_controller_period.py::test_controller_period_after_close_is_rejected PASSED [ 59%]
    tests/test_simulator_forward.py::test_reset_refreshes_all_body_positions PASSED [ 60%]
    tests/test_simulator_forward.py::test_reset_refreshes_body_orientations PASSED [ 60%]
    tests/test_simulator_forward.py::test_reset_refreshes_all_site_positions PASSED [ 60%]
    tests/test_simulator_forward.py::test_reset_refreshes_collision_geometry_positions PASSED [ 60%]
    tests/test_simulator_forward.py::test_ball_world_position_matches_restored_freejoint PASSED [ 60%]
    tests/test_simulator_forward.py::test_target_marker_position_is_refreshed PASSED [ 60%]
    tests/test_simulator_forward.py::test_reset_recomputes_current_contacts PASSED [ 61%]
    tests/test_simulator_forward.py::test_reset_preserves_primary_state_after_forward PASSED [ 61%]
    tests/test_simulator_forward.py::test_reset_does_not_advance_simulation_time PASSED [ 61%]
    tests/test_simulator_forward.py::test_reset_does_not_call_mj_step PASSED [ 61%]
    tests/test_simulator_forward.py::test_reset_calls_forward_exactly_once PASSED [ 61%]
    tests/test_simulator_forward.py::test_forward_occurs_after_state_restoration PASSED [ 61%]
    tests/test_simulator_forward.py::test_applied_forces_remain_cleared_after_forward PASSED [ 62%]
    tests/test_simulator_forward.py::test_reset_keeps_model_and_data_objects PASSED [ 62%]
    tests/test_simulator_forward.py::test_repeated_resets_have_stable_derived_state PASSED [ 62%]
    tests/test_simulator_forward.py::test_rejected_seed_does_not_call_forward PASSED [ 62%]
    tests/test_simulator_forward.py::test_reset_before_loading_is_rejected PASSED [ 62%]
    tests/test_simulator_forward.py::test_reset_after_closure_is_rejected PASSED [ 62%]
    tests/test_simulator_forward.py::test_independent_simulators_do_not_share_refresh_state PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_new_simulator_does_not_load_model PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_explicit_load_creates_model_and_data PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_independent_simulators_have_independent_models PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_loading_one_simulator_does_not_load_another PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_model_access_before_load_rejected PASSED [ 63%]
    tests/test_simulator_lifecycle.py::test_data_access_before_load_rejected PASSED [ 64%]
    tests/test_simulator_lifecycle.py::test_duplicate_load_rejected PASSED   [ 64%]
    tests/test_simulator_lifecycle.py::test_failed_load_preserves_new_state PASSED [ 64%]
    tests/test_simulator_lifecycle.py::test_close_transitions_to_closed PASSED [ 64%]
    tests/test_simulator_lifecycle.py::test_close_before_load_is_safe PASSED [ 64%]
    tests/test_simulator_lifecycle.py::test_double_close_is_safe PASSED      [ 64%]
    tests/test_simulator_lifecycle.py::test_access_after_close_rejected PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_reload_after_close_rejected PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_reset_before_load_rejected PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_step_before_load_rejected PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_step_is_available_after_load PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_invalid_constructor_path_rejected[None] PASSED [ 65%]
    tests/test_simulator_lifecycle.py::test_invalid_constructor_path_rejected[17] PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_invalid_constructor_path_rejected[2.5] PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_invalid_constructor_path_rejected[True] PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_invalid_constructor_path_rejected[] PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_load_does_not_advance_physics PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_close_does_not_advance_physics PASSED [ 66%]
    tests/test_simulator_lifecycle.py::test_close_does_not_destroy_external_reference PASSED [ 67%]
    tests/test_simulator_lifecycle.py::test_instances_remain_independent_after_one_closes PASSED [ 67%]
    tests/test_simulator_reset.py::test_reset_restores_entire_primary_state PASSED [ 67%]
    tests/test_simulator_reset.py::test_reset_restores_all_seven_arm_joints PASSED [ 67%]
    tests/test_simulator_reset.py::test_reset_restores_both_finger_joints PASSED [ 67%]
    tests/test_simulator_reset.py::test_reset_restores_ball_translation_and_quaternion PASSED [ 67%]
    tests/test_simulator_reset.py::test_reset_restores_all_ball_velocities PASSED [ 68%]
    tests/test_simulator_reset.py::test_reset_restores_every_actuator_command PASSED [ 68%]
    tests/test_simulator_reset.py::test_reset_restores_target_position PASSED [ 68%]
    tests/test_simulator_reset.py::test_reset_restores_time_without_stepping PASSED [ 68%]
    tests/test_simulator_reset.py::test_reset_preserves_model_and_data_identity PASSED [ 68%]
    tests/test_simulator_reset.py::test_repeated_reset_is_idempotent PASSED  [ 68%]
    tests/test_simulator_reset.py::test_reset_snapshot_is_not_a_live_view PASSED [ 69%]
    tests/test_simulator_reset.py::test_reset_does_not_replace_other_instance_state PASSED [ 69%]
    tests/test_simulator_reset.py::test_reset_before_load_is_rejected PASSED [ 69%]
    tests/test_simulator_reset.py::test_reset_after_close_is_rejected PASSED [ 69%]
    tests/test_simulator_reset.py::test_seeded_reset_not_silently_accepted PASSED [ 69%]
    tests/test_simulator_reset.py::test_unsupported_seed_does_not_change_state PASSED [ 69%]
    tests/test_simulator_reset.py::test_reset_does_not_advance_physics PASSED [ 70%]
    tests/test_simulator_reset.py::test_reset_does_not_recompile_scene PASSED [ 70%]
    tests/test_simulator_reset.py::test_initial_snapshot_is_cleared_on_close PASSED [ 70%]
    tests/test_simulator_reset.py::test_snapshot_arrays_are_read_only PASSED [ 70%]
    tests/test_simulator_reset.py::test_step_then_reset_restores_original_state PASSED [ 70%]
    tests/test_simulator_stepping.py::test_one_step_advances_one_physics_timestep PASSED [ 70%]
    tests/test_simulator_stepping.py::test_time_advances_by_exact_substep_count[1] PASSED [ 71%]
    tests/test_simulator_stepping.py::test_time_advances_by_exact_substep_count[2] PASSED [ 71%]
    tests/test_simulator_stepping.py::test_time_advances_by_exact_substep_count[5] PASSED [ 71%]
    tests/test_simulator_stepping.py::test_time_advances_by_exact_substep_count[20] PASSED [ 71%]
    tests/test_simulator_stepping.py::test_time_advances_by_exact_substep_count[40] PASSED [ 71%]
    tests/test_simulator_stepping.py::test_stepping_twice_accumulates_physics_time PASSED [ 71%]
    tests/test_simulator_stepping.py::test_step_uses_loaded_model_timestep PASSED [ 72%]
    tests/test_simulator_stepping.py::test_step_advances_ball_under_gravity PASSED [ 72%]
    tests/test_simulator_stepping.py::test_no_control_preserves_existing_command PASSED [ 72%]
    tests/test_simulator_stepping.py::test_valid_control_vector_is_written PASSED [ 72%]
    tests/test_simulator_stepping.py::test_valid_gripper_control_is_not_treated_as_metres PASSED [ 72%]
    tests/test_simulator_stepping.py::test_numpy_array_and_list_controls_are_accepted PASSED [ 72%]
    tests/test_simulator_stepping.py::test_input_command_is_copied_not_modified PASSED [ 73%]
    tests/test_simulator_stepping.py::test_nonpositive_substeps_rejected_without_stepping[0] PASSED [ 73%]
    tests/test_simulator_stepping.py::test_nonpositive_substeps_rejected_without_stepping[-1] PASSED [ 73%]
    tests/test_simulator_stepping.py::test_nonpositive_substeps_rejected_without_stepping[-20] PASSED [ 73%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[True] PASSED [ 73%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[False] PASSED [ 73%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[1.0] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[2] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[None] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_invalid_substep_types_rejected[count5] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_wrong_control_shape_rejected[invalid0] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_wrong_control_shape_rejected[invalid1] PASSED [ 74%]
    tests/test_simulator_stepping.py::test_wrong_control_shape_rejected[invalid2] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_wrong_control_shape_rejected[invalid3] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_multidimensional_control_rejected[invalid0] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_multidimensional_control_rejected[invalid1] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_nonfinite_controls_rejected[invalid0] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_nonfinite_controls_rejected[invalid1] PASSED [ 75%]
    tests/test_simulator_stepping.py::test_nonfinite_controls_rejected[invalid2] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_non_numeric_control_vectors_rejected[invalid0] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_non_numeric_control_vectors_rejected[invalid1] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_non_numeric_control_vectors_rejected[invalid2] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_invalid_control_representations_rejected[True] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_invalid_control_representations_rejected[1.5] PASSED [ 76%]
    tests/test_simulator_stepping.py::test_invalid_control_representations_rejected[123] PASSED [ 77%]
    tests/test_simulator_stepping.py::test_invalid_control_representations_rejected[invalid3] PASSED [ 77%]
    tests/test_simulator_stepping.py::test_out_of_range_gripper_control_rejected PASSED [ 77%]
    tests/test_simulator_stepping.py::test_negative_gripper_control_rejected PASSED [ 77%]
    tests/test_simulator_stepping.py::test_invalid_control_does_not_partially_write PASSED [ 77%]
    tests/test_simulator_stepping.py::test_invalid_timestep_rejected[0.0] PASSED [ 77%]
    tests/test_simulator_stepping.py::test_invalid_timestep_rejected[-0.002] PASSED [ 78%]
    tests/test_simulator_stepping.py::test_invalid_timestep_rejected[nan] PASSED [ 78%]
    tests/test_simulator_stepping.py::test_invalid_timestep_rejected[inf] PASSED [ 78%]
    tests/test_simulator_stepping.py::test_model_data_identity_preserved_during_stepping PASSED [ 78%]
    tests/test_simulator_stepping.py::test_step_invokes_mujoco_exactly_n_times PASSED [ 78%]
    tests/test_simulator_stepping.py::test_invalid_control_never_invokes_mujoco_step PASSED [ 78%]
    tests/test_simulator_stepping.py::test_control_is_held_constant_across_substeps PASSED [ 79%]
    tests/test_simulator_stepping.py::test_reset_after_stepping_restores_initial_time_and_state PASSED [ 79%]
    tests/test_simulator_stepping.py::test_multiple_instances_advance_independently PASSED [ 79%]
    tests/test_simulator_stepping.py::test_step_before_load_is_rejected PASSED [ 79%]
    tests/test_simulator_stepping.py::test_step_after_close_is_rejected PASSED [ 79%]
    tests/test_simulator_stepping.py::test_step_does_not_create_viewer PASSED [ 79%]
    tests/test_simulator_stepping.py::test_none_control_does_not_attempt_array_conversion PASSED [ 80%]
    tests/test_simulator_transients.py::test_generalised_applied_forces_are_cleared PASSED [ 80%]
    tests/test_simulator_transients.py::test_external_body_forces_are_cleared PASSED [ 80%]
    tests/test_simulator_transients.py::test_solver_warmstart_is_cleared PASSED [ 80%]
    tests/test_simulator_transients.py::test_transients_are_cleared_together PASSED [ 80%]
    tests/test_simulator_transients.py::test_reset_restores_captured_actuator_activation PASSED [ 80%]
    tests/test_simulator_transients.py::test_reset_does_not_zero_valid_control_commands PASSED [ 81%]
    tests/test_simulator_transients.py::test_reset_restores_joint_state_and_clears_forces PASSED [ 81%]
    tests/test_simulator_transients.py::test_reset_preserves_ball_quaternion PASSED [ 81%]
    tests/test_simulator_transients.py::test_reset_restores_target_and_clears_forces PASSED [ 81%]
    tests/test_simulator_transients.py::test_reset_restores_time_and_clears_warmstart PASSED [ 81%]
    tests/test_simulator_transients.py::test_repeated_resets_do_not_reintroduce_forces PASSED [ 81%]
    tests/test_simulator_transients.py::test_two_simulators_have_independent_transients PASSED [ 82%]
    tests/test_simulator_transients.py::test_reset_keeps_model_and_data_identity PASSED [ 82%]
    tests/test_simulator_transients.py::test_reset_does_not_call_mj_step PASSED [ 82%]
    tests/test_simulator_transients.py::test_reset_refreshes_forward_state PASSED [ 82%]
    tests/test_simulator_transients.py::test_native_reset_is_called_once PASSED [ 82%]
    tests/test_simulator_transients.py::test_unsupported_seed_preserves_all_transients PASSED [ 82%]
    tests/test_simulator_transients.py::test_reset_after_close_is_rejected PASSED [ 83%]
    tests/test_simulator_transients.py::test_reset_does_not_modify_model_joint_limits PASSED [ 83%]
    tests/test_simulator_transients.py::test_no_viewer_is_required PASSED    [ 83%]
    tests/test_simulator_transients.py::test_corrupt_snapshot_rejected_before_native_reset PASSED [ 83%]
    tests/test_spawn_and_target.py::test_default_spawn_has_clearance_and_settles_without_explosive_correction[demo] PASSED [ 83%]
    tests/test_spawn_and_target.py::test_default_spawn_has_clearance_and_settles_without_explosive_correction[composed] PASSED [ 83%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[demo-0.54] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[demo-0.540001] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[demo-0.62] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[composed-0.54] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[composed-0.540001] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_touching_or_clear_spawn_is_accepted[composed-0.62] PASSED [ 84%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position0-table_top] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position1-table_top] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position2-floor] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position3-table_leg_front_left] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position4-table_leg_front_right] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position5-table_leg_back_left] PASSED [ 85%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[demo-position6-table_leg_back_right] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position0-table_top] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position1-table_top] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position2-floor] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position3-table_leg_front_left] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position4-table_leg_front_right] PASSED [ 86%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position5-table_leg_back_left] PASSED [ 87%]
    tests/test_spawn_and_target.py::test_overlapping_configured_spawn_is_rejected[composed-position6-table_leg_back_right] PASSED [ 87%]
    tests/test_spawn_and_target.py::test_spawn_inside_robot_is_rejected PASSED [ 87%]
    tests/test_spawn_and_target.py::test_target_is_visible_fixed_site_without_collision_geometry[demo] PASSED [ 87%]
    tests/test_spawn_and_target.py::test_target_is_visible_fixed_site_without_collision_geometry[composed] PASSED [ 87%]
    tests/test_spawn_and_target.py::test_target_tracks_configured_position[demo] PASSED [ 87%]
    tests/test_spawn_and_target.py::test_target_tracks_configured_position[composed] PASSED [ 88%]
    tests/test_spawn_and_target.py::test_target_overlap_cannot_change_ball_trajectory[position0] PASSED [ 88%]
    tests/test_spawn_and_target.py::test_target_overlap_cannot_change_ball_trajectory[position1] PASSED [ 88%]
    tests/test_spawn_and_target.py::test_target_overlap_cannot_change_ball_trajectory[position2] PASSED [ 88%]
    tests/test_spawn_and_target.py::test_composition_rejects_robot_overlapping_default_ball_spawn PASSED [ 88%]
    tests/test_table.py::test_table_is_fixed_and_supports_ball PASSED        [ 88%]
    tests/test_table.py::test_leg_contacts_ball_below_tabletop[table_leg_front_left] PASSED [ 89%]
    tests/test_table.py::test_leg_contacts_ball_below_tabletop[table_leg_front_right] PASSED [ 89%]
    tests/test_table.py::test_leg_contacts_ball_below_tabletop[table_leg_back_left] PASSED [ 89%]
    tests/test_table.py::test_leg_contacts_ball_below_tabletop[table_leg_back_right] PASSED [ 89%]
    tests/test_table.py::test_space_under_table_is_open PASSED               [ 89%]
    tests/test_table.py::test_tabletop_contact_boundary[right-overlap] PASSED [ 89%]
    tests/test_table.py::test_tabletop_contact_boundary[right-clearance] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[left-overlap] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[left-clearance] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[back-overlap] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[back-clearance] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[front-overlap] PASSED [ 90%]
    tests/test_table.py::test_tabletop_contact_boundary[front-clearance] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[underside-overlap] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[underside-clearance] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-ne-overlap] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-ne-clearance] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-nw-overlap] PASSED [ 91%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-nw-clearance] PASSED [ 92%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-se-overlap] PASSED [ 92%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-se-clearance] PASSED [ 92%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-sw-overlap] PASSED [ 92%]
    tests/test_table.py::test_tabletop_contact_boundary[corner-sw-clearance] PASSED [ 92%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0--1-table_leg_front_left] PASSED [ 92%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0--1-table_leg_front_right] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0--1-table_leg_back_left] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0--1-table_leg_back_right] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0-1-table_leg_front_left] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0-1-table_leg_front_right] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0-1-table_leg_back_left] PASSED [ 93%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-0-1-table_leg_back_right] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1--1-table_leg_front_left] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1--1-table_leg_front_right] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1--1-table_leg_back_left] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1--1-table_leg_back_right] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1-1-table_leg_front_left] PASSED [ 94%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1-1-table_leg_front_right] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1-1-table_leg_back_left] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[overlap-1-1-table_leg_back_right] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0--1-table_leg_front_left] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0--1-table_leg_front_right] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0--1-table_leg_back_left] PASSED [ 95%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0--1-table_leg_back_right] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0-1-table_leg_front_left] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0-1-table_leg_front_right] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0-1-table_leg_back_left] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-0-1-table_leg_back_right] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1--1-table_leg_front_left] PASSED [ 96%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1--1-table_leg_front_right] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1--1-table_leg_back_left] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1--1-table_leg_back_right] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1-1-table_leg_front_left] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1-1-table_leg_front_right] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1-1-table_leg_back_left] PASSED [ 97%]
    tests/test_table.py::test_leg_contact_boundaries[clearance-1-1-table_leg_back_right] PASSED [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[0.44-0] PASSED   [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[-0.44-0] PASSED  [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[0-0.34] PASSED   [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[0--0.34] PASSED  [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[0.44-0.34] PASSED [ 98%]
    tests/test_table.py::test_ball_settles_near_table_edges[-0.44--0.34] PASSED [ 99%]
    tests/test_table.py::test_ball_settles_near_table_edges[-0.44-0.34] PASSED [ 99%]
    tests/test_table.py::test_ball_settles_near_table_edges[0.44--0.34] PASSED [ 99%]
    tests/test_table.py::test_ball_outside_table_falls_to_floor[0.51-0] PASSED [ 99%]
    tests/test_table.py::test_ball_outside_table_falls_to_floor[-0.51-0] PASSED [ 99%]
    tests/test_table.py::test_ball_outside_table_falls_to_floor[0-0.41] PASSED [ 99%]
    tests/test_table.py::test_ball_outside_table_falls_to_floor[0--0.41] PASSED [100%]
    
    ============================= 602 passed in 3.87s ==============================

