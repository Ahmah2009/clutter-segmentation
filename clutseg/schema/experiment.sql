create table experiment (
    id integer primary key autoincrement,
    name varchar(255) unique not null,
    paramset_id integer not null references paramset(id),
    response_id integer default null references response(id),
    train_set varchar(255) not null,
    test_set varchar(255) not null,
    time datetime,
    vcs_commit varchar(255),
    human_note varchar(255) DEFAULT(''),
    machine_note varchar(255) DEFAULT(''),
    batch varchar(255) DEFAULT(''),
    skip boolean default 0,
    flags integer default 0
);

