create table experiment (
    id integer primary key autoincrement,
    name varchar(255) unique,
    paramset_id integer not null references paramset(id),
    response_id integer default null references response(id),
    train_set varchar(255) not null,
    test_set varchar(255) not null,
    time datetime,
    vcs_commit varchar(255),
    skip boolean default 0
);

