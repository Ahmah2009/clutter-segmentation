
create table experiment (
    paramset_id integer not null references paramset(id),
    response_id integer references response(id),
    train_set varchar(255) not null,
    test_set varchar(255) not null,
    time datetime not null
);

