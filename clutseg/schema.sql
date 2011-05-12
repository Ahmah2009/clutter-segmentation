---
---
---

create table experiments (
    train_pms_fe_id integer references pms_fe(id),
    recog_pms_fe_id integer references pms_fe(id),
    detect_pms_match_id integer references pms_match(id),
    detect_pms_guess_id integer references pms_guess(id),
    locate_pms_match_id integer references pms_match(id),
    locate_pms_guess_id integer references pms_guess(id),
    pms_clutseg_id integer references pms_clutseg(id),
    train_set varchar(255),
    test_set varchar(255),
    time date,
    response_id integer unique
);

create table response (
    id integer unique,
    value float,
    -- max_rotation_error float,
    -- max_translation_error float
    -- tp
    -- fp
    -- fn
    -- tn
);

create table pms_fe (
    id integer unique,
    detector_type varchar(255), 
    extractor_type varchar(255), 
    descriptor_type varchar(255),
    threshold float,
    min_features integer,
    max_features integer
);

create table pms_match (
    id integer unique,
    matcher_type varchar(255),
    knn integer,
    do_ratio_test boolean,
    ratio_threshold float
);

create table pms_guess (
    id integer unique,
    min_inliers integer,
    max_projection_error float
);


create table pms_clutseg (
    id integer unique,
    acceptance_threshold float,
    ranking varchar(255)
);

