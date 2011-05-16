
insert into response (value) values (0.78);
insert into pms_clutseg (accept_threshold, ranking) values (15, "InliersRanking");
insert into pms_match (matcher_type, knn, do_ratio_test, ratio_threshold) values ("LSH-BINARY", 3, 1, 0.8);
insert into pms_match (matcher_type, knn, do_ratio_test, ratio_threshold) values ("LSH-BINARY", 3, 0, null);
insert into pms_guess (ransac_iterations_count, min_inliers_count, max_projection_error) values (100, 5, 12);
insert into pms_guess (ransac_iterations_count, min_inliers_count, max_projection_error) values (300, 15, 10);
insert into pms_fe (detector_type, extractor_type, descriptor_type, threshold, min_features, max_features, octaves) values ("FAST", "multi-scale", "rBRIEF", 40, null, null, 3);
insert into pms_fe (detector_type, extractor_type, descriptor_type, threshold, min_features, max_features, octaves) values ("FAST", "multi-scale", "rBRIEF", 40, null, null, 3);
insert into paramset (train_pms_fe_id, recog_pms_fe_id, detect_pms_match_id, detect_pms_guess_id, locate_pms_match_id, locate_pms_guess_id, recog_pms_clutseg_id) values (1, 2, 1, 1, 2, 2, 1);
insert into experiment (paramset_id, response_id, train_set, test_set, time) values (1, 1, "train", "test", "2010-01-02 23:22:19");

