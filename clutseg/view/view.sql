drop view if exists view_experiment_response;
drop view if exists view_experiment_note;
drop view if exists view_experiment_runtime;
drop view if exists view_experiment_error;
drop view if exists view_experiment_detect_roc;
drop view if exists view_experiment_scores;
drop view if exists view_experiment_detect_sipc;
drop view if exists view_experiment_locate_sipc;
drop view if exists view_experiment_all;

create view view_experiment_response as
    select experiment.id as experiment_id,
        experiment.name as experiment_name, response.*
        from experiment
        join response on experiment.response_id = response.id;

create view view_experiment_note as
    select id as experiment_id,
        human_note,
        machine_note,
        skip
    from experiment;

create view view_experiment_runtime as
    select experiment_id,
        train_runtime,
        test_runtime
    from view_experiment_response;

create view view_experiment_error as
    select experiment_id,
        experiment_name,
        succ_rate,
        avg_angle_err,
        avg_trans_err,
        avg_succ_angle_err,
        avg_succ_trans_err
    from view_experiment_response;

create view view_experiment_detect_roc as
    select experiment_id,
        experiment_name,
        succ_rate,
        detect_tp,
        detect_fp,
        detect_fn,
        detect_tn,
        1.0 * detect_tp / (detect_tp + detect_fn) as detect_tp_rate,
        1.0 * detect_fp / (detect_fp + detect_tn) as detect_fp_rate
    from view_experiment_response;
 
create view view_experiment_scores as
    select experiment_id,
        experiment_name,
        succ_rate,
        value,
        (1.0 * detect_sipc_acc_score / detect_sipc_objects) as detect_sipc,
        (0.5 * locate_sipc_cscore + 0.25 * locate_sipc_rscore + 0.25 * locate_sipc_tscore) / locate_sipc_frames as locate_sipc
    from view_experiment_response;
  
create view view_experiment_detect_sipc as
    select experiment_id,
        experiment_name,
        succ_rate,
        (1.0 * detect_sipc_acc_score / detect_sipc_objects) as detect_sipc,
        detect_sipc_acc_score,
        detect_sipc_objects
    from view_experiment_response;
  
create view view_experiment_locate_sipc as
    select experiment_id,
        experiment_name,
        succ_rate,
        (0.5 * locate_sipc_cscore + 0.25 * locate_sipc_rscore + 0.25 * locate_sipc_tscore) / locate_sipc_frames as locate_sipc,
        locate_sipc_cscore,
        locate_sipc_rscore,
        locate_sipc_tscore
    from view_experiment_response;
 
create view view_experiment_all as
    select e.id as experiment_id,
        e.name as experiment_name,
        tf.detector_type as train_detector_type,
        tf.extractor_type as train_extractor_type,
        tf.descriptor_type as train_descriptor_type,
        tf.threshold as train_threshold,
        tf.min_features as train_min_features,
        tf.max_features as train_max_features,
        tf.octaves as train_octaves,
        rf.detector_type as recog_detector_type,
        rf.extractor_type as recog_extractor_type,
        rf.descriptor_type as recog_descriptor_type,
        rf.threshold as recog_threshold,
        rf.min_features as recog_min_features,
        rf.max_features as recog_max_features,
        rf.octaves as recog_octaves,
        dm.matcher_type as detect_matcher_type, 
        dm.knn as detect_knn, 
        dm.do_ratio_test as detect_do_ratio_test,
        dm.ratio_threshold as detect_do_ratio_threshold,
        dg.ransac_iterations_count as detect_ransac_iterations_count,
        dg.min_inliers_count as detect_min_inliers_count,
        dg.max_projection_error as detect_max_projection_error,
        lm.matcher_type as locate_matcher_type, 
        lm.knn as locate_knn, 
        lm.do_ratio_test as locate_do_ratio_test,
        lm.ratio_threshold as locate_do_ratio_threshold,
        lg.ransac_iterations_count as locate_ransac_iterations_count,
        lg.min_inliers_count as locate_min_inliers_count,
        lg.max_projection_error as locate_max_projection_error,
        c.*,
        r.*
    from experiment e
    join paramset p on e.paramset_id = p.id
    join pms_fe tf on p.train_pms_fe_id = tf.id
    join pms_fe rf on p.recog_pms_fe_id = rf.id
    join pms_match dm on p.detect_pms_match_id = dm.id
    join pms_guess dg on p.detect_pms_guess_id = dg.id
    join pms_match lm on p.locate_pms_match_id = lm.id
    join pms_guess lg on p.locate_pms_guess_id = lg.id
    join pms_clutseg c on p.recog_pms_clutseg_id = c.id
    join response r on e.response_id = r.id;
        
