
create table pms_fe (
    id integer primary key autoincrement,
    detector_type varchar(255), 
    extractor_type varchar(255), 
    descriptor_type varchar(255),
    threshold float,
    min_features integer,
    max_features integer,
    octaves integer
);

