    
create table response (
    id integer primary key autoincrement,
    value float not null,
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
    detect_tp_rate float not null,
    detect_fp_rate float not null,
    avg_locate_matches float not null,
    avg_locate_inliers float not null,
    avg_locate_choice_matches float not null,
    avg_locate_choice_inliers float not null
);

