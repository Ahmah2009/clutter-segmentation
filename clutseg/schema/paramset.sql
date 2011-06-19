
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

