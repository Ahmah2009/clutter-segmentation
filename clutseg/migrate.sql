begin transaction;
    alter table response rename to tmp_response;
    
    create table response (
        id integer primary key autoincrement,
        value float not null,
        -- SIPC score 
        detect_sipc_acc_score float not null,
        detect_sipc_objects integer not null,
        -- modified SIPC score for single object
        refine_sipc_rscore float not null,
        refine_sipc_tscore float not null,
        refine_sipc_cscore float not null,
        refine_sipc_frames integer not null,
        -- the following statistics depend only on the one single choice made
        avg_angle_err float not null,
        avg_succ_angle_err float not null,
        avg_trans_err float not null,
        avg_succ_trans_err float not null,
        avg_angle_sq_err float not null,
        avg_succ_angle_sq_err float not null,
        avg_trans_sq_err float not null,
        avg_succ_trans_sq_err float not null,
        succ_rate float not null,
        mislabel_rate float not null,
        none_rate float not null,
        avg_keypoints float not null,
        avg_detect_guesses float not null,
        avg_detect_matches float not null,
        avg_detect_inliers float not null,
        avg_detect_choice_matches float not null,
        avg_detect_choice_inliers float not null,
        detect_tp integer not null,
        detect_fp integer not null,
        detect_fn integer not null,
        detect_tn integer not null,
        -- the following statistics are only valid if locating step is performed
        avg_refine_guesses float not null,
        avg_refine_matches float not null,
        avg_refine_inliers float not null,
        avg_refine_choice_matches float not null,
        avg_refine_choice_inliers float not null,
        -- timing stats --
        train_runtime float not null,
        test_runtime float not null
    );

    insert into response (
        id,
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
        test_runtime)
    select
        id,
        value,
        detect_sipc_acc_score,
        detect_sipc_objects,
        locate_sipc_rscore,
        locate_sipc_tscore,
        locate_sipc_cscore,
        locate_sipc_frames,
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
        avg_locate_guesses,
        avg_locate_matches,
        avg_locate_inliers,
        avg_locate_choice_matches,
        avg_locate_choice_inliers,
        train_runtime,
        test_runtime
    from tmp_response;

    drop table tmp_response;

    alter table paramset rename to tmp_paramset;

    create table paramset (
        id integer primary key autoincrement,
        train_pms_fe_id integer not null references pms_fe(id),
        recog_pms_fe_id integer not null references pms_fe(id),
        detect_pms_match_id integer not null references pms_match(id),
        detect_pms_guess_id integer not null references pms_guess(id),
        refine_pms_match_id integer not null references pms_match(id),
        refine_pms_guess_id integer not null references pms_guess(id),
        recog_pms_clutseg_id integer not null references pms_clutseg(id)
    );

    insert into paramset (
        id,
        train_pms_fe_id,
        recog_pms_fe_id,
        detect_pms_match_id,
        detect_pms_guess_id,
        refine_pms_match_id,
        refine_pms_guess_id,
        recog_pms_clutseg_id)
    select
        id,
        train_pms_fe_id,
        recog_pms_fe_id,
        detect_pms_match_id,
        detect_pms_guess_id,
        locate_pms_match_id,
        locate_pms_guess_id,
        recog_pms_clutseg_id
    from tmp_paramset;

    drop table tmp_paramset;

    commit;
