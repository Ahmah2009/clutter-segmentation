
insert into response (
    value,
    detect_sipc_acc_score,
    detect_sipc_objects,
    refine_sipc_rscore,
    refine_sipc_tscore,
    refine_sipc_cscore,
    refine_sipc_frames,
    avg_angle_err,
    avg_succ_angle_err,
    avg_trans_err,
    avg_succ_trans_err,
    avg_angle_sq_err,
    avg_succ_angle_sq_err,
    avg_trans_sq_err,
    avg_succ_trans_sq_err,
    succ_rate,
    mislabel_rate,
    none_rate,
    avg_keypoints,
    avg_detect_guesses,
    avg_detect_matches,
    avg_detect_inliers,
    avg_detect_choice_matches,
    avg_detect_choice_inliers,
    detect_tp,
    detect_fp,
    detect_fn,
    detect_tn,
    avg_refine_guesses,
    avg_refine_matches,
    avg_refine_inliers,
    avg_refine_choice_matches,
    avg_refine_choice_inliers,
    train_runtime,
    test_runtime
    ) values
    (0.78, 36, 63, 0.25, 0.75, 1.0, 21, 0.34, 0.08, 0.12, 0.02, 0.56, 0.15, 0.53, 0.03, 0.63, 0.05, 0.15, 913.0, 55, 652.3, 9.2, 211.9, 13.3, 35, 5, 10, 40, 33, 802.1, 29.8, 802.1, 39.8, 320.5, 214.8);
insert into pms_clutseg (accept_threshold, ranking) values (15, "InliersRanking");
insert into pms_match (matcher_type, knn, do_ratio_test, ratio_threshold) values ("LSH-BINARY", 3, 1, 0.8);
insert into pms_match (matcher_type, knn, do_ratio_test, ratio_threshold) values ("LSH-BINARY", 3, 0, null);
insert into pms_guess (ransac_iterations_count, min_inliers_count, max_projection_error) values (100, 5, 12);
insert into pms_guess (ransac_iterations_count, min_inliers_count, max_projection_error) values (300, 15, 10);
insert into pms_fe (detector_type, extractor_type, descriptor_type, threshold, scale_factor, min_features, max_features, n_features, octaves) values ("FAST", "multi-scale", "rBRIEF", 40, 1.2, null, null, 5000, 3);
insert into pms_fe (detector_type, extractor_type, descriptor_type, threshold, scale_factor, min_features, max_features, n_features, octaves) values ("FAST", "multi-scale", "rBRIEF", 40, 1.2, null, null, 5000, 3);
insert into paramset (train_pms_fe_id, recog_pms_fe_id, detect_pms_match_id, detect_pms_guess_id, refine_pms_match_id, refine_pms_guess_id, recog_pms_clutseg_id) values (1, 2, 1, 1, 2, 2, 1);
insert into experiment (paramset_id, response_id, name, train_set, test_set, time, vcs_commit) values (1, 1, "some-experiment", "train", "test", "2010-01-02 23:22:19", "ccb521d7307ef27a65ab82f297be80390b5599bb");

