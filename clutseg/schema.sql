---
---
---

create table experiment (
    paramset_id integer not null references paramset(id),
    response_id integer not null references response(id),
    train_set varchar(255) not null,
    test_set varchar(255) not null,
    time datetime not null
);

create table paramset (
    id integer primary key,
    train_pms_fe_id integer not null references pms_fe(id),
    recog_pms_fe_id integer not null references pms_fe(id),
    detect_pms_match_id integer not null references pms_match(id),
    detect_pms_guess_id integer not null references pms_guess(id),
    locate_pms_match_id integer not null references pms_match(id),
    locate_pms_guess_id integer not null references pms_guess(id),
    recog_pms_clutseg_id integer not null references pms_clutseg(id)
);

create table response (
    id integer primary key,
    value float not null
    -- max_rotation_error float,
    -- max_translation_error float
    -- tp
    -- fp
    -- fn
    -- tn
);

create table pms_fe (
    id integer primary key,
    detector_type varchar(255), 
    extractor_type varchar(255), 
    descriptor_type varchar(255),
    threshold float,
    min_features integer,
    max_features integer,
    octaves integer
);

create table pms_match (
    id integer primary key,
    matcher_type varchar(255),
    knn integer,
    do_ratio_test boolean,
    ratio_threshold float
);

create table pms_guess (
    id integer primary key,
    ransac_iterations_count integer not null,
    min_inliers integer not null,
    max_projection_error float not null,
    -- constraints
    check (ransac_iterations_count >= 0),
    check (min_inliers >= 0),
    check (max_projection_error >= 0)
);

create table pms_clutseg (
    id integer primary key,
    acceptance_threshold float,
    ranking varchar(255)
);

