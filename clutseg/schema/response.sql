    
create table response (
    id integer primary key autoincrement,
    value float not null,
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
    avg_detect_matches float not null,
    avg_detect_inliers float not null,
    avg_detect_choice_matches float not null,
    avg_detect_choice_inliers float not null,
    detect_tp integer not null,
    detect_fp integer not null,
    detect_fn integer not null,
    detect_tn integer not null,
    -- the following statistics are only valid if locating step is performed
    avg_locate_matches float not null,
    avg_locate_inliers float not null,
    avg_locate_choice_matches float not null,
    avg_locate_choice_inliers float not null
);

create view response_detect_roc as
    select experiment.id as experiment_id,
        detect_tp as tp,
        detect_fp as fp,
        detect_fn as fn,
        detect_tn as tn,
        1.0 * detect_tp / (detect_tp + detect_fn) as tp_rate,
        1.0 * detect_fp / (detect_fp + detect_tn) as fp_rate
    from response join experiment on experiment.response_id = response.id;
