
create table experiment (
    id integer primary key autoincrement,
    paramset_id integer not null references paramset(id),
    response_id integer default null references response(id),
    train_set varchar(255) not null,
    test_set varchar(255) not null,
    time datetime not null,
    vcs_commit varchar(255) 
);

