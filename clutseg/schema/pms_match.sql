
create table pms_match (
    id integer primary key autoincrement,
    matcher_type varchar(255),
    knn integer,
    do_ratio_test boolean,
    ratio_threshold float
);

